package com.neocoretechs.robocore.propulsion;

//import org.apache.commons.logging.Log;
import java.io.File;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CountDownLatch;

import org.ros.concurrent.CancellableLoop;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import com.neocoretechs.robocore.MegaControl;
import com.neocoretechs.robocore.RosArrayUtilities;
import com.neocoretechs.robocore.PID.IMUSetpointInfo;
import com.neocoretechs.robocore.PID.MotionPIDController;
import com.neocoretechs.robocore.PID.MotorPIDController;
import com.neocoretechs.robocore.config.Props;
import com.neocoretechs.robocore.config.Robot;
import com.neocoretechs.robocore.config.RobotInterface;


/**
 * We are fusing data from the IMU, any joystick input, and other autonomous controls to affect
 * propulsion and issue status updates based on IMU data edge conditions that may indicate problems.
 * Subscribing to:
 * sensor_msgs/Joy (joystick information boxed as an array of buttons and axis values common to standard controllers)
 * sensor_msgs/MagneticField (3 axis magnetic flux information most likely from IMU)
 * sensor_msgs/Temperature (most likely from IMU)
 * sensor_msgs/Imu (absolutely from IMU, contains heading information roll/pitch/yaw in Euler and Quaternion, pre-fused)
 *
 * Publishing to:
 * absolute/cmd_vel (command velocity channel receiving absolute left and right motor speed values)
 * robocore/status (status update channel to receive information on alerts, etc, derived from IMU and other sensors)
 * 
 * Published on absolute/cmd_vel are the values from 0-1000 that represent actual power to each differential drive wheel.
 * Subscribers to absolute/cmd_vel include the process that manages serial data to the motor controller, possibly on a separate computer.
 * Publishers on robocore/status provide status updates relating to IMU extremes that are picked up by the VoxHumana voice synth or others,
 * again most probably on a completely separate single board computer within the robot.
 *
 * To reduce incoming data, Two sets of flags and a threshold value are used to determine if:
 * a) The values have changed from last message
 * b) The value exceed a threshold deemed noteworthy
 * 
 * We take steps to align the joystick reference frame to the following:
 *      / 0 \
 *   90   |  -90
 *    \   |   /
 *     180,-180
 * The ratio of arc lengths of radius speed and speed+WHEELBASE, and turn angle theta from course difference 
 * becomes the ratio of speeds at each wheel.
 * When in autonomous mode we use the same steering method for large granularity turns. At mid granularity we compute a solution
 * using a right triangle whose base is speed and whose hypotenuse is computed from scaled speed and course deviation. For the short leg,
 * course deviation and half wheeltrack form a chord with radius of WHEELBASE/2 and angle of course deviation: 2*(wheelbase/2)*sin(deviation/2). 
 * The ratio of hypotenuse to base is our distance and hence speed for each wheel as we do with the arc solution.
 * finally at the smallest granularity a PID controller takes over to maintain critical crosstrack damping.
 * This methodology is analogous to a human driver, who upon leaving the road and entering median, must compute a wide arc to re-enter
 * the roadway, then perform adjustments that bring them to a tangent of the center of the lane, and from there perform a series
 * of small corrections to stay on course. Regardless of the amount the machine is knocked off course it can find its way back to the
 * proper heading through a circuitous route gradually refined to require only small correction.
 * 
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2017,2018,2019,2020
 *
 */
public class MotionController extends AbstractNodeMain {
	private static boolean DEBUG = false;
	private String host;
	private InetSocketAddress master;
	private CountDownLatch awaitStart = new CountDownLatch(1);

	float rangetop, rangebot;
	double pressure, temperature;
	double[] mag = {0, 0, 0};
	double[] eulers = {0, 0, 0};
	//TwistInfo twistInfo;
	float last_theta;
	
	static final float TWOPI = (float) (Math.PI * 2.0f);
	public static boolean isShock = false;
	public static boolean isOverShock = false;
	
	// deltas. 971, 136, 36 relatively normal values. seismic: last value swings from rangeTop -40 to 140
	public static float[] SHOCK_BASELINE = { 971.0f, 136.0f, 36.0f};

	public static float[] SHOCK_THRESHOLD = {1000.0f,1000.0f,1000.0f}; 
	public static boolean isMag = false;
	public static boolean isOverMag = false;
	public static short[] MAG_THRESHOLD = {-1,-1,-1};
	//public static boolean isMag = false;
	public static int PRESSURE_THRESHOLD = 100000; // pressure_meas is millibars*100 30in is 1014 milli
	public static boolean isOverPressure = false;
	public static boolean isPressure = false;
	public static long lastPressureNotification = 0; // time so we dont just keep yapping about the weather
	public static int TEMPERATURE_THRESHOLD = Props.toInt("TemperatureThreshold");//40 C 104 F
	public static boolean isTemperature = false;
	public static boolean isOverTemp = false;
	public static boolean isMoving = false;
	public static boolean isNav = false;
	public static boolean isRangeUpperFront = false;
	public static boolean isRangeLowerFront = false;
	public static boolean isVision = false; // machine vision recognition event
	public static final float TRIANGLE_THRESHOLD = Props.toFloat("MaxIMUDeviationDegrees"); // Point at which geometric solution begins/disengages
	public static final float PID_THRESHOLD  = TRIANGLE_THRESHOLD/2; // point at which PID engages/disengages
	// If we call the wheelbase 1000, it implies that at maximum speed of 1000 it take 2 robot radii to turn 90 degrees
	// This is because the radius of the inner wheel is from speed 0 to speed max, and the outer is from speed 0 to speed max plus wheelbase.
	// Correspondingly at speed 1 the arc describing the inner wheel is 1 and the inner wheel turns at 1, or in place 
	// and the arc describing the outer wheel has a radius equivalent to the wheelbase and the 
	// outer wheel turns at the robot wheelbase through 90 degrees.
	public static float WHEELBASE = 1000.0f;

	
	boolean hasData = false; // have we received any feedback from callback?
	boolean init = true;
	static boolean holdBearing = false; // hold steering to current bearing

	static float speedL, speedR, SPEEDSCALE;

	static long lastTime = System.currentTimeMillis();
	static int SampleTime = 100; //.1 sec
	static float outMin, outMax;
	static boolean inAuto = false;
	static boolean outputNeg = false; // used to prevent integral windup by resetting ITerm when crossing track
	static boolean wasPid = true; // used to determine crossing from geometric to PID to preload integral windup
	static boolean LEDCamlightIsOn = false;
	
	Object rngMutex1 = new Object();
	Object rngMutex2 = new Object();
	Object navMutex = new Object();
	Object magMutex = new Object();
	
	RobotInterface robot;
	
	/**
	 * We really only use these methods if we want to pull remapped params out of command line or do
	 * some special binding, otherwise the default uses the ROS_HOSTNAME environment or the remapped __ip:= and __master:=
	 * @param host
	 * @param master
	 */
	public MotionController(String host, InetSocketAddress master) {
		this.host = host;
		this.master = master;
		NodeConfiguration nc = build();
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, nc);
	    if( DEBUG ) {
	    	System.out.println("Bringing up MotionControl with host and master:"+host+" "+master);
	    }   
	}
	
	public MotionController(String[] args) {
		CommandLineLoader cl = new CommandLineLoader(Arrays.asList(args));
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, cl.build());
	    if( DEBUG ) {
	    	System.out.println("Bringing up MotionControl with args:"+args[0]+" "+args[1]+" "+args[2]);
	    }
	}
	
	public MotionController() {
		if(DEBUG)
			System.out.println("CTOR robot...");
		robot = new Robot();
		if(DEBUG)
			System.out.println("CTOR build robot"+robot);
	}
	
	
	/**
	 * Create NodeConfiguration.
	 */
	public NodeConfiguration build()  {
	  //NodeConfiguration nodeConfiguration = NodeConfiguration.copyOf(Core.getInstance().nodeConfiguration);
	  NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(host, master);
	  nodeConfiguration.setParentResolver(NameResolver.newFromNamespace("/"));
	  nodeConfiguration.setRosRoot(null); // currently unused
	  nodeConfiguration.setRosPackagePath(new ArrayList<File>());
	  nodeConfiguration.setMasterUri(master);
	  nodeConfiguration.setNodeName(getDefaultNodeName());
	  return nodeConfiguration;
	}
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("pubsubs_motion");
	}
	
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		//robot = new Robot();
		//System.out.println("onStart build robot...");
		//System.out.println("onStart robot"+robot);
		//final Log log = connectedNode.getLog();
		Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();	
		if( remaps.containsKey("__speedlimit") ) {
			robot.getLeftSpeedSetpointInfo().setMaximum(Float.parseFloat(remaps.get("__speedlimit")));
			robot.getRightSpeedSetpointInfo().setMaximum(Float.parseFloat(remaps.get("__speedlimit")));
		}
		if( remaps.containsKey("__kp") )
			robot.getMotionPIDController().setKp(Float.parseFloat(remaps.get("__kp")));
		if( remaps.containsKey("__kd") )
			robot.getMotionPIDController().setKd(Float.parseFloat(remaps.get("__kd")));
		//final Publisher<geometry_msgs.Twist> mopub = connectedNode.newPublisher("cmd_vel", geometry_msgs.Twist._TYPE);
		// statpub has status alerts that may come from sensors.
		if(robot.getDiffDrive().isIndoor())
			SPEEDSCALE = 100.0f;
		else
			SPEEDSCALE = 1000.0f;
		
		final Publisher<diagnostic_msgs.DiagnosticStatus> statpub =
				connectedNode.newPublisher("robocore/status", diagnostic_msgs.DiagnosticStatus._TYPE);
		final Publisher<std_msgs.Int32MultiArray> velpub =
				connectedNode.newPublisher("absolute/cmd_vel", std_msgs.Int32MultiArray._TYPE);
		final Publisher<std_msgs.Int32MultiArray> trigpub =
				connectedNode.newPublisher("absolute/cmd_periph1", std_msgs.Int32MultiArray._TYPE);
		final Publisher<geometry_msgs.Twist> twistpub = 
				connectedNode.newPublisher("cmd_vel", geometry_msgs.Twist._TYPE);
		
		final geometry_msgs.Twist twistmsg = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Twist._TYPE);
		final geometry_msgs.Vector3 angular = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
		final geometry_msgs.Vector3 linear = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
		final geometry_msgs.Quaternion orientation =  connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE); 
		
		Subscriber<sensor_msgs.Joy> subsrange = connectedNode.newSubscriber("/sensor_msgs/Joy", sensor_msgs.Joy._TYPE);
		Subscriber<sensor_msgs.Imu> subsimu = connectedNode.newSubscriber("/sensor_msgs/Imu", sensor_msgs.Imu._TYPE);
		//Subscriber<sensor_msgs.Range> subsrangetop = connectedNode.newSubscriber("UpperFront/sensor_msgs/Range", sensor_msgs.Range._TYPE);
		//Subscriber<sensor_msgs.Range> subsrangebot = connectedNode.newSubscriber("LowerFront/sensor_msgs/Range", sensor_msgs.Range._TYPE);
		Subscriber<sensor_msgs.MagneticField> subsmag = connectedNode.newSubscriber("/sensor_msgs/MagneticField", sensor_msgs.MagneticField._TYPE);
		Subscriber<sensor_msgs.Temperature> substemp = connectedNode.newSubscriber("/sensor_msgs/Temperature", sensor_msgs.Temperature._TYPE);
		/*
		Subscriber<sensor_msgs.PointCloud> subsrange = connectedNode.newSubscriber("robocore/kinect", sensor_msgs.PointCloud._TYPE);

		subsrange.addMessageListener(new MessageListener<sensor_msgs.PointCloud>() {
			@Override
			public void onNewMessage(sensor_msgs.PointCloud message) {
				List<geometry_msgs.Point32> cloud = message.getPoints();
				if( DEBUG )
					System.out.println("Kinect.."+cloud.size()+" points..");
				try {
			
				} catch (Throwable e) {
					e.printStackTrace();
				}	
			}
		});
		*/
		if(DEBUG)
			System.out.println("PREPUBLISHING robot"+robot);
		//
		// Joystick data will have array of axis and buttons, axis[0] and axis[2] are left stick x,y axis[1] and axis[3] are right.
		// The code calculates the theoretical speed for each wheel in the 0-1000 scale or SPEEDSCALE based on target point vs IMU yaw angle.
		// If the joystick chimes in the target point becomes the current course minus relative stick position,
		// and the speed is modified by the Y value of the stick.
		// Button presses cause rotation in place or emergency stop or cause the robot to hold to current course, using the IMU to 
		// correct for deviation an wheel slippage.
		// To turn, we are going to calculate the arc lengths of the inner wheel and outer wheel based on speed we are presenting by stick y
		// (speed) forming the radius of the arc and the offset of stick angle x,y degrees from 0 added to current heading forming theta.
		// Theta may also be formed by button press and the difference in current heading and ongoing IMU headings for bearing on a forward course.
		// We are then going to assume that the distance each wheel has to travel represents a ratio that is also the ratio
		// of speeds of each wheel, time and speed being distance and all, and the fact that both wheels, being attached to the robot,
		// have to arrive at essentially the same time after covering the desired distance based on desired speed.
		// The net effect that as speed increases the turning radius also increases, as the radius is formed by speed (Y of stick) scaled by 1000
		// in the case of the inner wheel, and inner wheel plus 'effective robot width' or WHEELBASE as the outer wheel arc radius to traverse.
		// So we have the theta formed by stick and IMU, and 2 radii formed by stick y and WHEELBASE and we generate 2 arc lengths that are taken
		// as a ratio that is multiplied by the proper wheel depending on direction to reduce power in one wheel to affect turn.
		// The ratio of arc lengths depending on speed and turn angle becomes the ratio of speeds at each wheel.
		// The above method is used for interactive control via joystick and for large granularity correction in autonomous mode.
		// The same technique is used in autonomous mode for finer correction by substituting the base of a right triangle as the speed 
		// for the inner arc and the hypotenuse computed by the base and chord formed from course deviation and half wheelbase for the outer arc.
		// The triangle solution uses radius in the forward direction, rather than at right angles with WHEELBASE as the arcs do, to bring
		// us into refined tangents to the crosstrack. At final correction a PID algorithm is used to maintain fine control.
		//
		subsrange.addMessageListener(new MessageListener<sensor_msgs.Joy>() {
			@Override
			public void onNewMessage(sensor_msgs.Joy message) {
				float[] axes = message.getAxes();
				int[] buttons = message.getButtons();
				// check for emergency stop, on X or A or green or lower button
				if( buttons[6] != 0 ) {
					System.out.println("**EMERGENCY STOP FROM VELOCITY "+axes[2]);
					angular.setX(-1);
					angular.setY(-1);
					angular.setZ(-1);
					linear.setX(-1);
					linear.setY(-1);
					linear.setZ(-1);
					twistmsg.setAngular(angular);
					twistmsg.setLinear(linear);
					twistpub.publish(twistmsg);
					return;
				}
				// If the button square or circle is depressed, rotate in place at stick position Y speed
				if( buttons[7] != 0 ) { // left pivot
					speedR = -axes[2] * SPEEDSCALE;
					speedL = -speedR;
					// set it up to send
					ArrayList<Integer> speedVals = new ArrayList<Integer>(6);
					speedVals.add(robot.getDiffDrive().getControllerSlot()); //controller slot
					speedVals.add(robot.getDiffDrive().getLeftWheelChannel()); // controller slot channel
					speedVals.add((int)speedL);
					speedVals.add(robot.getDiffDrive().getControllerSlot()); // controller slot
					speedVals.add(robot.getDiffDrive().getRightWheelChannel()); // controller slot channel
					speedVals.add((int)speedR);
					velpub.publish(setupPub(connectedNode, speedVals,"Controller slot/channel/val","Controller slot/channel/val value"));
					System.out.printf("Stick Left pivot speedL=%f|speedR=%f\n",speedL,speedR);
					return;
				} else {
					if( buttons[5] != 0 ) { // right pivot
						speedL = -axes[2] * SPEEDSCALE;
						speedR = -speedL;
						// set it up to send
						ArrayList<Integer> speedVals = new ArrayList<Integer>(6);
						speedVals.add(robot.getDiffDrive().getControllerSlot()); //controller slot
						speedVals.add(robot.getDiffDrive().getLeftWheelChannel()); // controller slot channel
						speedVals.add((int)speedL);
						speedVals.add(robot.getDiffDrive().getControllerSlot()); // controller slot
						speedVals.add(robot.getDiffDrive().getRightWheelChannel()); // controller slot channel
						speedVals.add((int)speedR);
						velpub.publish(setupPub(connectedNode, speedVals,"Controller slot/channel/val","Controller slot/channel/val value"));
						System.out.printf("Stick Right pivot speedL=%f|speedR=%f\n",speedL,speedR);
						return;
					}
				}
				// get radian measure of left stick x,y
				float radians = (float) Math.atan2(axes[2], axes[0]);
				float stickDegrees = (float) (radians * (180.0 / Math.PI)); // convert to degrees
				// horizontal axis is 0 to -180 degrees, we want 0 at the top
				stickDegrees += 90;
				stickDegrees = (stickDegrees > 180.0 ? stickDegrees -= 90.0 : -stickDegrees);
				// we have absolute stickdegrees, offset the current IMU by that quantity to get new desired course
				float offsetDegrees = (float) (eulers[0] - stickDegrees);
			    // reduce the angle  
			    offsetDegrees =  offsetDegrees % 360;
			    //
			    if (offsetDegrees <= -180.0)
			         offsetDegrees += 360.0;
			    if (offsetDegrees >= 180)  
			         offsetDegrees -= 360;
			    // force it to be the positive remainder, so that 0 <= angle < 360  
			    //offsetDegrees = (offsetDegrees + 360) % 360;  
			    // force into the minimum absolute value residue class, so that -180 < angle <= 180  
			    //if (offsetDegrees > 180)  
			    //      offsetDegrees -= 360;
			      
				//if( DEBUG )
				//	System.out.println("Axes:"+axes[0]+","+axes[2]+" deg:"+bearingDegrees+" rad:"+radians);
			    // Button 4 - triangle. Establish setpoint for autonomous course following.
				// If we are pressing the button, dont update the setpoint with IMU yaw, this has the effect of locking
				// the course onto the absolute track relative to the way the robot is pointing offset by stick position
				// rather than a continuous update of stick offset added to current orientation. so if you want the robot to track
				// straight forward on its current orientation, hold triangle/top and push stick forward or back.
			    if( buttons[4] != 0 ) {   
					// is it initial button press? then log current bearing of IMU to hold to
					if( !holdBearing ) {
						holdBearing = true;
						outputNeg = false;
						robot.getIMUSetpointInfo().setDesiredTarget((float) eulers[0]); // Setpoint is desired target yaw angle from IMU,vs heading minus the joystick offset from 0
						robot.getMotionPIDController().clearPID();
						wasPid = true; // start with PID control until we get out of tolerance
						System.out.println("Stick absolute deg set:"+eulers[0]);	
					}
			    } else {
			    	holdBearing = false;
			    	robot.getIMUSetpointInfo().setDesiredTarget(offsetDegrees); // joystick setting becomes desired target yaw angle heading and IMU is disregarded
			    }
	
				robot.getIMUSetpointInfo().setTarget((float) eulers[0]); //eulers[0] is Input: target yaw angle from IMU
				//
				// perform the PID processing
				//
				((MotionPIDController) robot.getMotionPIDController()).Compute(robot.getIMUSetpointInfo());
				
				// Speed may be plus or minus, this determines a left or right turn as the quantity is added to desired speed
				// of wheels left and right. The speed for each wheel is modified by desired turn radius, which is based on speed,
				// or on the computed derivation from desired course if a straight line course is set.
				// axes[2] is y, value is -1 to 0 to 1, and for some reason forward is negative on the stick
				// scale it from -1 to 0 to 1 to -1000 to 0 to 1000, or the value of SPEEDSCALE which is our speed control range 
				speedR = -axes[2] * SPEEDSCALE;
				speedL = speedR;
				// PTerm has the positive or negative difference in degrees (left or right offset)
				// between IMU heading and joystick, we have to calc the scaling factors for wheel speed then apply.
				// We determine the radius of the turn potential by the forward speed desired (joystick Y)
				// For instance if at 0 we can spin at radius .5 of wheelbase if necessary, while at speed 500 out of 1000
				// we can turn at radius 1, which is 90 degrees with inner wheel stationary, and at 1000 out of 1000
				// we turn inner radius 1 and outer radius 2 robot widths as defined by WHEELBASE, which is a gradual arc.
				// degrees/180 * PI * (r + WHEELBASE) = outer wheel arc length
				// degrees/180 * PI * r = inner wheel arc length
				// we compare the arc lengths as a ratio.
				// We use the speed in the range of the stick Y range (0 to 1) times a scale, and establish that as a scale factor to 
				// add to wheelbase, so at max speed we extend the radius by one robot length; joystick y + WHEELBASE making 
				// inner radius 1 'robot length unit' and outer radius 1 + WHEELBASE units.
				// We dont really need absolute measurements, we just assign the values based on stick Y and a scale factor
				// and the same scale factor to 1 'robot width unit'. We are multiplying the stick Y by 1000 and WHEELBASE by 1000
				// but in reality we are using radius 0 to 1, and then 0 to 1 plus WHEELBASE scaled to finer granularity.
				// This assumes that at half speed we can still make a 90 degree turn.
				// We use value of IMU vs desired course to turn left or right via
				// a plus or minus value from the Compute method set in the variable called PTerm.
				float radius = Math.abs(axes[2]) * SPEEDSCALE;
				float arcin, arcout;
				//
				// If holding an inertially guided course, check for the tolerance of error proportion
				// from setpoint to current heading, if within a lower tolerance, use the pid values of the
				// currently computed course deviation to
				// alter the motor speed to reduce cross track error and become critically damped.
				// If we are at the higher tolerance, compute a solution based on triangles that brings us on a tangent course
				// to the crosstrack. The base of a right triangle is the speed plus half wheelbase, the hypotenuse is computed from
				// the chord of the magnitude error offset from track and the base (speed). The ratio of base to hypotenuse is used
				// based on required distance traveled for each wheel and the opposite wheel is slowed by that proportion
				// just as we do for a solution of arcs.
				// The chord leg of the triangle is computed based on half wheelbase as radius via 2*R*sin(theta/2) with theta
				// being crosstrack deviation.
				//
				if( holdBearing ) {
					if(DEBUG)
					System.out.printf("Inertial Setpoint=%f | Hold=%b ", robot.getIMUSetpointInfo().getDesiredTarget(), holdBearing);
					// In auto
					if( Math.abs(robot.getMotionPIDController().getError()) <= TRIANGLE_THRESHOLD ) {
						if( robot.getMotionPIDController().getError() < 0.0f ) { // decrease left wheel power goal
							// If in lower bound use PID, between lower and middle use triangle solution, above that use arc
							if( Math.abs(robot.getMotionPIDController().getError()) <= PID_THRESHOLD ) {
								rightPid(radius);
							} else {
								rightAngle(radius);
							}
						} else {
							if( robot.getMotionPIDController().getError() > 0.0f ) { // decrease right wheel power goal
								if( robot.getMotionPIDController().getError() <= PID_THRESHOLD) {
									leftPid(radius);
								} else {
									leftAngle(radius);
								}
							} else {
								// we are critically damped, set integral to 0
								//ITerm = 0;
								robot.getMotionPIDController().setITerm(0);
							}
						}
						if(DEBUG)
						System.out.printf("<="+TRIANGLE_THRESHOLD+" degrees Speed=%f|IMU=%f|speedL=%f|speedR=%f|Hold=%b\n",radius,eulers[0],speedL,speedR,holdBearing);
					} else {
						// Exceeded tolerance of triangle solution, proceed to polar geometric solution in arcs
						robot.getMotionPIDController().setITerm(0);//ITerm = 0;
						wasPid = false;
						arcin = (float) (Math.abs(robot.getMotionPIDController().getError()/360.0) * (2.0 * Math.PI) * radius);
						arcout = (float) (Math.abs(robot.getMotionPIDController().getError()/360.0) * (2.0 * Math.PI) * (radius + WHEELBASE));
						// error is difference in degrees between setpoint and input
						if( robot.getMotionPIDController().getError() < 0.0f )
							speedL *= (arcin/arcout);
						else
							if( robot.getMotionPIDController().getError() > 0.0f )
								speedR *= (arcin/arcout);
						if(DEBUG)
						System.out.printf(">"+TRIANGLE_THRESHOLD+" degrees Speed=%f|IMU=%f|arcin=%f|arcout=%f|speedL=%f|speedR=%f|Hold=%b\n",radius,eulers[0],arcin,arcout,speedL,speedR,holdBearing);
					}
				} else {
					// manual steering mode, use tight radii and a human integrator
					arcin = (float) (Math.abs(robot.getMotionPIDController().getError()/360.0) * (2.0 * Math.PI) * radius);
					arcout = (float) (Math.abs(robot.getMotionPIDController().getError()/360.0) * (2.0 * Math.PI) * (radius + WHEELBASE));
					// error is difference in degrees between setpoint and input
					// Stick position will override any IMU tolerances
					if( stickDegrees != -180.0 && stickDegrees != 180 && stickDegrees != 0.0) { // if we want to go straight forward or backwards dont bother computing any ratios
						if( robot.getMotionPIDController().getError() < 0.0f )
							speedL *= (arcin/arcout);
						else
							if( robot.getMotionPIDController().getError() > 0.0f )
								speedR *= (arcin/arcout);
					}
					if(DEBUG)
					System.out.printf("Stick deg=%f|Offset deg=%f|IMU=%f|arcin=%f|arcout=%f|speedL=%f|speedR=%f|Hold=%b\n",stickDegrees,offsetDegrees,eulers[0],arcin,arcout,speedL,speedR,holdBearing);
				}
				if(DEBUG)
				System.out.printf("%s | Hold=%b\n",robot.getMotionPIDController().toString(), holdBearing);
				//
				// set it up to send down the publishing pipeline
				//
				ArrayList<Integer> speedVals = new ArrayList<Integer>(6);
				speedVals.add(robot.getDiffDrive().getControllerSlot()); //controller slot
				speedVals.add(robot.getDiffDrive().getLeftWheelChannel()); // controller slot channel
				speedVals.add((int)speedL);
				speedVals.add(robot.getDiffDrive().getControllerSlot()); //controller slot
				speedVals.add(robot.getDiffDrive().getRightWheelChannel()); // controller slot channel
				speedVals.add((int)speedR);
				velpub.publish(setupPub(connectedNode, speedVals,"Controller slot/channel/val","Controller slot/channel/val value"));
				try {
					Thread.sleep(5);
				} catch (InterruptedException e) {}
				//-------------------
				// Above cases handle all steering and motor control and button press for automated
				// control of course following by button press.
				// See if the triggers were activated. Axes[4] and axes[5] are the left and right triggers.
				// Check them and see if either one was depressed. If so, scale them to the -1000 to 1000
				// SPEEDSCALE constant (or whatever the SPEEDSCALE constant is, we presume its set at 1000)
				// for the majority of downstream processing. In the case of PWM, we are going to scale this
				// fr0m -1000,1000 to 0,2000 since controls such as LED dont have a negativer or 'reverse' value.
				// Actually, could be potentially destructive to reverse polarity as a motor does, so we are
				// sure to scale it to the positive range downstream. We are going to publish the scaled
				// values to absolute/cmd_periph1 and let the downstream processing handle further scaling
				// if necessary. If we reach an off state of -1, we want to send it anyway to turn off the LED, 
				// hence the boolean checks.
				// LEDCameraIlluminatorControl:4
				// LEDCameraIlluminatorSlot:1
				// LEDCameraIlluminatorChannel:1
				//-------------------
				if(axes[Props.toInt("LEDCameraIlluminatorControl")] != -1 || LEDCamlightIsOn) {//|| axes[5] != -1) {
					if(axes[Props.toInt("LEDCameraIlluminatorControl")] == -1) {
						if(LEDCamlightIsOn) {
							ArrayList<Integer> triggerVals = new ArrayList<Integer>(3);
							triggerVals.add(Props.toInt("LEDCameraIlluminatorSlot")); //controller slot
							triggerVals.add(Props.toInt("LEDCameraIlluminatorChannel")); // controller slot channel
							triggerVals.add(0);
							if(DEBUG)
							System.out.println("LEDCameraIlluminatorControl turning off LED");
							trigpub.publish(setupPub(connectedNode, triggerVals,"LEDCameraIlluminatorControl","LEDCameraIlluminatorControl"));
							try {
								Thread.sleep(5);
							} catch (InterruptedException e) {}
						}
						LEDCamlightIsOn = false;
					} else {
						LEDCamlightIsOn = true;
						ArrayList<Integer> triggerVals = new ArrayList<Integer>(3);
						triggerVals.add(Props.toInt("LEDCameraIlluminatorSlot")); //controller slot
						triggerVals.add(Props.toInt("LEDCameraIlluminatorChannel")); // controller slot channel
						triggerVals.add(Integer.valueOf((int)axes[Props.toInt("LEDCameraIlluminatorControl")]));
						if(DEBUG)
						System.out.printf("LEDCameraIlluminatorControl = %d\n",triggerVals.get(2));
						trigpub.publish(setupPub(connectedNode, triggerVals,"LEDCameraIlluminatorControl","LEDCameraIlluminatorControl"));
						try {
							Thread.sleep(5);
						} catch (InterruptedException e) {}
					}
				}
			} // onMessage from Joystick controller, with all the axes[] and buttons[]
			
			/**
			 * Increase power of right wheel based on PID results
			 * If we hit max we are going to decrease power to left wheel by splitting the difference
			 * of leftover speed on right applied to left wheel decrease. whether these are equivalent in reality is
			 * left to evaluation.
			 * @param speed the scaled velocity
			 */
			private void rightPid(float speed) {
				/*
				if( !outputNeg ) { // prevent integral windup by checking when track is crossed via sign of course diff changing
					ITerm = 0;
				}
				outputNeg = true;
				// see if we are entering from the geometric realm, if so preload integral
				if( !wasPid ) {
					wasPid = true;
					// get rid of old integral and replace with our preload windup
					// Since we are in negative offsets we will subtract the maximum offset that is threshold value 
					pidOutput = PTerm + DTerm + (-PID_THRESHOLD * ki);
					// Now set up integral to reflect transition state
					ITerm = -PID_THRESHOLD;
				}
				*/
				// Reduce speed of left wheel to affect turn, speed reduction may result in negative values turning wheel backwards if necessary
				// scale it by speed
				speedL -= ( Math.abs(robot.getMotionPIDController().getOutput()) * (speed/350) );
				/*
				// goal is to increase power of right wheel
				// We scale it by speed with the assumption that at full speed,
				// the values from PID represent the ones necessary to work at top speed of 1000 or SPEEDLIMIT.
				speedR += ( Math.abs(pidOutput) * (Math.abs(axes[2])*3.2) );
				// if we cap at max, use the difference to retard opposite
				if( speedR > SPEEDLIMIT ) {
					// over limit, split the difference
					float speeDif = speedR - SPEEDLIMIT;
					speedR = SPEEDLIMIT;
					// left is decreased by the amount right went over max
					speedL -= speeDif;
					if( speedL < 0.0 ) speedL = 0.0f;
					// and the robot is pivoting at max speed if everything was capped
				}
				*/
			}
			/**
			 * Increase power of left wheel base don PID results. If we max out we are going to apply the leftover
			 * power over maximum to decreasing the right wheel speed.
			 * @param speed The scaled velocity
			 */
			private void leftPid(float speed) {
				/*
				if( outputNeg ) {
					ITerm = 0;
				}
				outputNeg = false;
				// see if we are entering from the geometric realm, if so preload integral
				if( !wasPid ) {
					wasPid = true;
					// get rid of old integral and replace with our preload windup
					// Since we are in positive offsets we will add the maximum threshold value for windup.
					pidOutput = PTerm + DTerm + (PID_THRESHOLD * ki);
					// Now set up integral to reflect transition state
					ITerm = PID_THRESHOLD;
				}
				*/
				// Reduce speed of right wheel to affect turn, speed reduction may result in negative values turning wheel backwards if necessary
				// scale it by speed
				speedR -= ( Math.abs(robot.getMotionPIDController().getOutput()) * (speed/350) );
				/*
				// goal is net increase of left wheel power
				// We scale it by speed with the assumption that at full speed,
				// the values from PID represent the ones necessary to work at top speed of 1000 or SPEEDLIMIT.
				speedL +=( Math.abs(pidOutput) * (Math.abs(axes[2])*3.2) );
				if( speedL > SPEEDLIMIT ) {
					// over limit, split the difference
					float speeDif = speedL - SPEEDLIMIT;
					speedL = SPEEDLIMIT;
					speedR -= speeDif;
					if( speedR < 0.0) speedR = 0.0f;
				}
				*/
				
			}
			/**
			 * Apply a solution in triangles as we do with solution in arcs to reduce left wheel speed.
			 * This presupposes we are within tolerance.
			 * @param radius velocity-based value of turn radius component
			 */
			private void rightAngle(float radius) {
				// Mitigate integral windup
				robot.getMotionPIDController().setITerm(0);//ITerm = 0;
				wasPid = false;
				// compute chord of course deviation, this is short leg of triangle.
				// chord is 2Rsin(theta/2) ( we convert to radians first)
				double chord = 2 * ((WHEELBASE/2)*2.7) * Math.sin((Math.abs(robot.getMotionPIDController().getError()/360.0) * (2.0 * Math.PI))/2);
				// make the base of the triangle radius , the do a simple pythagorean deal and take the 
				// square root of sum of square of base and leg
				double hypot = Math.sqrt((chord*chord) + ((radius+(WHEELBASE/2))*(radius+(WHEELBASE/2))));
				// decrease the power on the opposite side by ratio of base to hypotenuse, since one wheel needs to 
				// travel the length of base and the other needs to travel length of hypotenuse
				if(DEBUG)
				System.out.printf(" RIGHTANGLE=%f|chord=%f|hypot=%f|radius/hypot=%f|Hold=%b\n",radius,chord,hypot,((radius+(WHEELBASE/2))/hypot),holdBearing);
				speedL *= ((radius+(WHEELBASE/2))/hypot);
			}
			/**
			 * Apply a solution in triangles as we do with solution in arcs to reduce right wheel speed.
			 * This presupposes we are within tolerance.
			 * @param radius Velocity based component of turning radius
			 */
			private void leftAngle(float radius) {
				// Mitigate integral windup
				robot.getMotionPIDController().setITerm(0);//ITerm = 0;
				wasPid = false;
				// compute chord of course deviation, this is short leg of triangle.
				// chord is 2Rsin(theta/2), R is half wheeltrack so we assume our robot is roughly 'square'
				double chord = 2 * ((WHEELBASE/2)*2.7) * Math.sin((Math.abs(robot.getMotionPIDController().getError()/360.0) * (2.0 * Math.PI))/2);
				// make the base of the triangle radius , the do a simple pythagorean deal and take the 
				// square root of sum of square of base and leg
				double hypot = Math.sqrt((chord*chord) + ((radius+(WHEELBASE/2))*(radius+(WHEELBASE/2))));
				// decrease the power on the opposite side by ratio of base to hypotenuse, since one wheel needs to 
				// travel the length of base and the other needs to travel length of hypotenuse
				if(DEBUG)
				System.out.printf(" LEFTANGLE=%f|chord=%f|hypot=%f|radius/hypot=%f|Hold=%b\n",radius,chord,hypot,((radius+(robot.getDiffDrive().getWheelTrack()/2))/hypot),holdBearing);
				speedR *= ((radius+(WHEELBASE/2))/hypot);
			}
		});
		
		subsimu.addMessageListener(new MessageListener<sensor_msgs.Imu>() {
			@Override
			public void onNewMessage(sensor_msgs.Imu message) {
				synchronized(navMutex) {
					orientation.setX(message.getOrientation().getX());
					orientation.setY(message.getOrientation().getY());
					orientation.setZ(message.getOrientation().getZ());
					orientation.setW(message.getOrientation().getW());
					eulers = message.getOrientationCovariance();
					if(DEBUG) {
					System.out.println("Nav:Orientation X:"+orientation.getX()+" Y:"+orientation.getY()+" Z:"+orientation.getZ()+" W:"+orientation.getW());
					System.out.println("Nav:Eulers "+eulers[0]+" "+eulers[1]+" "+eulers[2]);
					}
					try {
					if( message.getAngularVelocity().getX() != angular.getX() ||
						message.getAngularVelocity().getY() != angular.getY() ||
						message.getAngularVelocity().getZ() != angular.getZ() ||
						message.getLinearAcceleration().getX() != linear.getX() ||
						message.getLinearAcceleration().getY() != linear.getY() ||
						message.getLinearAcceleration().getZ() != linear.getZ()) {
							angular.setX(message.getAngularVelocity().getX());
							angular.setY(message.getAngularVelocity().getY());
							angular.setZ(message.getAngularVelocity().getZ());
							linear.setX(message.getLinearAcceleration().getX());
							linear.setY(message.getLinearAcceleration().getY());
							linear.setZ(message.getLinearAcceleration().getZ());
							isNav = true;
							if( SHOCK_THRESHOLD[0] != -1 ) {
								if( Math.abs(linear.getX()-SHOCK_BASELINE[0]) > SHOCK_THRESHOLD[0] ) {
									isOverShock = true;
								}
								if( Math.abs(linear.getY()-SHOCK_BASELINE[1]) > SHOCK_THRESHOLD[1] ) {
									isOverShock = true;
								}
								if( Math.abs(linear.getZ()-SHOCK_BASELINE[2]) > SHOCK_THRESHOLD[2] ) {
									isOverShock = true;
								}
							}
							if(DEBUG) {
							System.out.printf("Nav:Angular X:%f | Angular Y:%f | Angular Z:%f",angular.getX(),angular.getY(),angular.getZ());
							System.out.printf("Nav:Linear X:%f | Linear Y:%f | Linear Z:%f",linear.getX(),linear.getY(),linear.getZ());
							}
					} else
						isNav = false;
				} catch (Throwable e) {
					isNav = false;
					System.out.println("Nav subs exception:"+e.getMessage());
					e.printStackTrace();
				}
				}
			}

		});
		/*
		subsrangetop.addMessageListener(new MessageListener<sensor_msgs.Range>() {
		@Override
		public void onNewMessage(sensor_msgs.Range message) {
			if( DEBUG )
				System.out.println("New range msg:"+message);
			synchronized(rngMutex1) {
				try {
					if( message.getRange() != rangetop) {
						rangetop = message.getRange();
						isRangeUpperFront = true;
						System.out.println(" Range Top:"+rangetop);
					} else
						isRangeUpperFront = false;
				} catch (Throwable e) {
					isRangeUpperFront = false;
					System.out.println("Range top subs exception:"+e.getMessage());
					e.printStackTrace();
				}
			}
		}
		});

		subsrangebot.addMessageListener(new MessageListener<sensor_msgs.Range>() {
		@Override
		public void onNewMessage(sensor_msgs.Range message) {
			if( DEBUG )
				System.out.println("New range msg:"+message);
			synchronized(rngMutex2) {
				try {
					if( message.getRange() != rangebot) {
						rangebot = message.getRange();
						isRangeLowerFront = true;
						System.out.println(" Range Bottom:"+rangebot);
					} else
						isRangeLowerFront = false;
				} catch (Throwable e) {
					isRangeLowerFront = false;
					System.out.println("Range bottom subs exception:"+e.getMessage());
					e.printStackTrace();
				}
			}
		}
		});
		*/
		subsmag.addMessageListener(new MessageListener<sensor_msgs.MagneticField>() {
			@Override
			public void onNewMessage(sensor_msgs.MagneticField message) {
				if( DEBUG )
					System.out.println("New mag msg:"+message);
				geometry_msgs.Vector3 mag3 = message.getMagneticField();
				synchronized(magMutex) {
				if( mag3.getX() != mag[0] || mag3.getY() != mag[1] || mag3.getZ() != mag[2]) {
					mag[0] = mag3.getX();
					mag[1] = mag3.getY();
					mag[2] = mag3.getZ();
					isMag = true;
					if(DEBUG)
					System.out.println("Magnetic field:"+mag[0]+" "+mag[1]+" "+mag[2]);
					if( MAG_THRESHOLD[0] != -1 ) {
						if( mag[0] > MAG_THRESHOLD[0] && mag[1] > MAG_THRESHOLD[1] && mag[2] > MAG_THRESHOLD[2] ) {
							isOverMag = true;
						}
					}
				} else
					isMag = false;
				}
			}
		});
		
		substemp.addMessageListener(new MessageListener<sensor_msgs.Temperature>() {
			@Override
			public void onNewMessage(sensor_msgs.Temperature message) {
				if( DEBUG )
					System.out.println("New temp msg:"+message);
				if(temperature != message.getTemperature()) {
					temperature = message.getTemperature();
					if(DEBUG)
					System.out.println(" Temp:"+temperature);
					isTemperature = true;
					if( temperature > TEMPERATURE_THRESHOLD )
						isOverTemp = true;
				} else
					isTemperature = false;
			}
		});
		
		// tell the waiting constructors that we have registered publishers
		awaitStart.countDown();
		
		//----------------------------------------
		// Begin publishing loop
		//----------------------------------------
		connectedNode.executeCancellableLoop(new CancellableLoop() {
			private int sequenceNumber;

			@Override
			protected void setup() {
				sequenceNumber = 0;
				robot.getIMUSetpointInfo().setPrevErr(0.0f); // 0 degrees yaw
				robot.getIMUSetpointInfo().setMinimum(-TRIANGLE_THRESHOLD); //  degree minimum integral windup
				robot.getIMUSetpointInfo().setMaximum(TRIANGLE_THRESHOLD); // maximum degree windup
				robot.getMotionPIDController().clearPID();
				WHEELBASE = Props.toFloat("MaxSpeedValue");
				System.out.println("POST setup robot:"+robot);
				// default kp,ki,kd;
				//SetTunings(5.55f, 1.0f, 0.5f); // 5.55 scales a max +-180 degree difference to the 0 1000,0 -1000 scale
				//SetOutputLimits(0.0f, SPEEDLIMIT); when pid controller created, max is specified
			}

			@Override
			protected void loop() throws InterruptedException {	
			    try {
					awaitStart.await();
				    if( DEBUG )
				    	System.out.println("Pub/subs registered..");
				} catch (InterruptedException e) {}
				/*
				hasMoved = motorControlListener.move2DRelative(np.getGyros()[0] , courseOffset, linearMove
						, np.getTimeVal(), np.getAccs(), np.getRanges());
					//System.out.println("Robot should have Moved to "+(robotTheta *= 57.2957795)+" degrees"); // to degrees		
				 */
				
				synchronized(rngMutex1) {
					if(isRangeUpperFront) {
						isRangeUpperFront = false;
					}
				}
				synchronized(rngMutex2) {
					if(isRangeLowerFront) {

						isRangeLowerFront = false;
					}
				}
	
				synchronized(navMutex) {
					if(isNav) {
						isNav = false;
						//System.out.println("Shock:"+accs[0]+" "+accs[1]+" "+accs[2]);
					}
				}
				
				// publish messages to status listener if applicable
				
				if( isOverMag ) {
					isOverMag = false;
					System.out.println("Mag EXCEEDS threshold..");
					diagnostic_msgs.DiagnosticStatus statmsg = statpub.newMessage();
					statmsg.setName("magnetic");
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.WARN);
					statmsg.setMessage("Magnetic anomaly detected");
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					statpub.publish(statmsg);
					Thread.sleep(1);
				}
				
				if( isOverShock && !isMoving) {
					System.out.println("OVERSHOCK!");
					diagnostic_msgs.DiagnosticStatus statmsg = statpub.newMessage();
					isOverShock = false;
					statmsg.setName("shock");
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.WARN);
					statmsg.setMessage("Accelerometer shock warning");
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					statpub.publish(statmsg);
					Thread.sleep(1);
				}
				
					
				if( isOverPressure ) { // check for dropping
						diagnostic_msgs.DiagnosticStatus statmsg = statpub.newMessage();
							//long meas = System.currentTimeMillis()-lastPressureNotification;
							//if( meas > 1000000) {
							//	isPress = true;
							//}
							//return;
							//}
						isOverPressure = false;
						statmsg.setName("pressure");
						statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.WARN);
						statmsg.setMessage("Atmospheric pressure warning");
						diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
						List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
						li.add(kv);
						statmsg.setValues(li);
						statpub.publish(statmsg);
						Thread.sleep(1);
				}
				
				if(isOverTemp) {
					if( DEBUG )
						System.out.println("DANGER Temp:"+temperature);
					diagnostic_msgs.DiagnosticStatus statmsg = statpub.newMessage();
					isOverTemp = false;
					statmsg.setName("temperature");
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.WARN);
					statmsg.setMessage("Temperature warning");
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					statpub.publish(statmsg);
					Thread.sleep(1);
				}
	
				Thread.sleep(10);
				++sequenceNumber;
				if( DEBUG )
					System.out.println("Sequence:"+sequenceNumber);
		}
	}); // cancellable loop

	}
	/**
	 * Move the robot to an absolute pose of 0-360 degrees and attempt to travel the target distance in the target time
	 * when the IMU and target coincide distance and time produce linear motion
	 * otherwise when turning distance determines the radius of the arc segment described by the difference
	 * of the current IMU and target and time becomes roughly the speed at which to make such traversal
	 * @param yawIMURads Yaw is delivered in radians -1.0 to 1.0
	 * @param yawTargetDegrees target final pose is in degrees for easier human input
	 * @param targetDistance target distance to travel in mm
	 * @param targetTime time in which to travel desired distance in seconds
	 * 	 * @return The Twist message with twistInfo.robotTheta as the value to turn (X,Y,deltaTheta,IMUTheta,wheelTheta,yawDegrees as well)
	 * @throws IOException 
	 */
	public synchronized TwistInfo moveRobotAbsolute(TwistInfo twistInfo, float yawIMURads, int yawTargetDegrees, int targetDistance) throws IOException {

	        //while (true) {

	                //leftWheel.targetMM = (float)(left_velocity) / clicks_per_mm;
	                //rightWheel.targetMM = (float)(right_velocity) / clicks_per_mm;

	                //avg_mm = (float) ((leftWheel.targetMM + rightWheel.targetMM) / 2.0);
	                //total_mm += avg_mm;
	                // wheel_theta is travel of each wheel with desired velocity, if same angle is 0, forward
	                //twistInfo.wheelTheta = (leftWheel.targetMM - rightWheel.targetMM) / wheelTrack;
	        		
	        		twistInfo.setWheelTheta((float)yawTargetDegrees * 0.0174532925f); // degr to radians
	                /* read the YAW value from the imu struct and convert to radians */
	                twistInfo.setImuTheta((float) (yawIMURads * Math.PI));
	                if( twistInfo.getImuTheta() < 0.0f ) {
	                	twistInfo.setImuTheta(twistInfo.getImuTheta() + TWOPI);
	                }
	                twistInfo.setYawDegrees(IMUSetpointInfo.yawDegrees(twistInfo.getImuTheta()));
	                
	                /* calculate rotation rate-of-change  */
	                twistInfo.setDeltaTheta(twistInfo.getImuTheta() - last_theta);
	                last_theta = twistInfo.getImuTheta();

	                // this will generate the +- value to turn us properly
	                twistInfo.setRobotTheta(twistInfo.getWheelTheta() - twistInfo.getImuTheta());
	                if( twistInfo.getRobotTheta() > TWOPI )
	                	twistInfo.setRobotTheta(twistInfo.getRobotTheta() - TWOPI);
	                if( twistInfo.getRobotTheta() < 0)
	                	twistInfo.setRobotTheta(twistInfo.getRobotTheta() + TWOPI);
	                    
	                twistInfo.setX(twistInfo.getX() + (float)(targetDistance * Math.sin(twistInfo.getRobotTheta()))); 
	                twistInfo.setY(twistInfo.getY() + (float)(targetDistance * Math.cos(twistInfo.getRobotTheta()))); 
	                // so X is the arc radius, normally we will set as a call to
	                // setMotorSpeed as 0 to turn in place or a value of arc radius
	                // modified by current speed to make a gentle curve.
	                // if theta is 0, move linear
	                //stasis(fabs(wheel_theta),fabs(imu_theta));
	                if( DEBUG )
	                	System.out.println(twistInfo);
	                setMotorArcSpeed(twistInfo.getRobotTheta(), yawIMURads);
	                return twistInfo;//twistInfo.robotTheta;
	        //}
	}
	
	/**
	 * Move the robot to an relative pose of 0-360 degrees and attempt to travel the target distance in the target time
	 * Here, targetDegrees can be negative
	 * when the IMU and target coincide distance and time produce linear motion
	 * otherwise when turning distance determines the radius of the arc segment described by the difference
	 * of the current IMU and target and time becomes roughly the speed at which to make such traversal
	 * @param yawIMURads Yaw is delivered in radians -1.0 to 1.0
	 * @param yawTargetDegrees target final pose is in degrees for easier human input and represents motion relative to current pose vs absolute position
	 * @param targetDistance target distance to travel in mm
	 * @param targetTime time in which to travel desired distance in seconds
	 * @return The Twist message with twistInfo.robotTheta as the value to turn (X,Y,deltaTheta,IMUTheta,wheelTheta,yawDegrees as well)
	 * @throws IOException 
	 */
	public synchronized TwistInfo moveRobotRelative(TwistInfo twistInfo, float yawIMURads, int yawTargetDegrees, int targetDistance) throws IOException {
	        //while (true) {

	                //leftWheel.targetMM = (float)(left_velocity) / clicks_per_mm;
	                //rightWheel.targetMM = (float)(right_velocity) / clicks_per_mm;

	                //avg_mm = (float) ((leftWheel.targetMM + rightWheel.targetMM) / 2.0);
	                //total_mm += avg_mm;
	                // wheel_theta is travel of each wheel with desired velocity, if same angle is 0, forward
	                //twistInfo.wheelTheta = (leftWheel.targetMM - rightWheel.targetMM) / wheelTrack;
	        		
	        		twistInfo.setWheelTheta((float)yawTargetDegrees * 0.0174532925f); // degr to radians
	                /* read the YAW value from the imu struct and convert to radians */
	                twistInfo.setImuTheta((float) (yawIMURads * Math.PI));
	                if( twistInfo.getImuTheta() < 0.0f ) {
	                	twistInfo.setImuTheta(twistInfo.getImuTheta() + TWOPI);
	                }
	                twistInfo.setYawDegrees(IMUSetpointInfo.yawDegrees(twistInfo.getImuTheta()));
	                
	                /* calculate rotation rate-of-change  */
	                twistInfo.setDeltaTheta(twistInfo.getImuTheta() - last_theta);
	                last_theta = twistInfo.getImuTheta();

	                twistInfo.setRobotTheta(twistInfo.getImuTheta() + twistInfo.getWheelTheta());
	                // this will generate the value to turn us properly
	                if( twistInfo.getRobotTheta() > TWOPI )
	                	twistInfo.setRobotTheta(twistInfo.getRobotTheta() - TWOPI);
	                if( twistInfo.getRobotTheta() < 0)
	                	twistInfo.setRobotTheta(twistInfo.getRobotTheta() + TWOPI);

	                twistInfo.setX(twistInfo.getX() + (float)(targetDistance * Math.sin(twistInfo.getRobotTheta()))); 
	                twistInfo.setY(twistInfo.getY() + (float)(targetDistance * Math.cos(twistInfo.getRobotTheta()))); 
	                // so X is the arc radius, normally we will set as a call to
	                // setMotorSpeed as 0 to turn in place or a value of arc radius
	                // modified by current speed to make a gentle curve.
	                // if theta is 0, move linear in x
	                // if x and theta not 0 its rotation about a point in space
	                //stasis(fabs(wheel_theta),fabs(imu_theta));
	                if( DEBUG )
	                	System.out.println(twistInfo);
	                // slot, channel 1, slot, channel 2, theta, yaw radians
	                setMotorArcSpeed(twistInfo.getWheelTheta(), yawIMURads);
	                return twistInfo;//twistInfo.robotTheta;
	        //}
	}
	
	/**
	 *  The function computes motor speeds.
	 *  results in leftWheel.targetSpeed and rightWheel.targetSpeed
	 * so X is the arc radius, normally we will set as a call to
	 * setMotorSpeed as 0 to turn in place or a value of arc radius
	 * modified by current speed to make a gentle curve.
	 * if theta is 0, move linear in x
	 * if x and theta not 0 its rotation about a point in space
	 * @param x The radius from the center point of the arc about which we are rotating, if 0 turn in place
	 * @param th The 2PI polar measure of arc segment we are traversing, if 0 pure forward/back motion
	 * @return int 2 element array of final wheel speed left, right
	 * @throws IOException 
	 */
	public synchronized int[] setMotorArcSpeed(float x, float th) throws IOException {

		float spd_left, spd_right;
  
		/* Reset the auto stop timer */
		//motorControl.lastMotorCommand = System.currentTimeMillis();

		/* Indicate that we are moving */
		//moving = true;

		if (x == 0) {
			// Turn in place, rotate the proper wheel forward, the other backward for a spin
			if( th < 0 ) { // left
				//spd_right = (float) (th * wheelTrack / 2.0);
				spd_right = th;
			} else {
				//spd_right = -((float) (th * wheelTrack / 2.0));
				spd_right = -th;
			}	
			spd_left = -spd_right;
		} else {
			if (th == 0) {	
				// Pure forward/backward motion
				spd_left = spd_right = x;
			} else {
				// Rotation about a point in space
				// Calculate Drive Turn output due to X input
				if (x > 0) {
				  // Forward
				  spd_left = ( th > 0 ) ? robot.getLeftSpeedSetpointInfo().getMaximum() : (robot.getLeftSpeedSetpointInfo().getMaximum() + th);
				  spd_right = ( th > 0 ) ? (robot.getRightSpeedSetpointInfo().getMaximum() - th) : robot.getRightSpeedSetpointInfo().getMaximum();
				} else {
				  // Reverse
				  spd_left = (th > 0 ) ? (robot.getLeftSpeedSetpointInfo().getMaximum() - th) : robot.getLeftSpeedSetpointInfo().getMaximum();
				  spd_right = (th > 0 ) ? robot.getRightSpeedSetpointInfo().getMaximum() : (robot.getRightSpeedSetpointInfo().getMaximum() + th);
				}

				// Scale Drive output due to X input (throttle)
				spd_left = spd_left * x / robot.getLeftSpeedSetpointInfo().getMaximum();
				spd_right = spd_right * x / robot.getRightSpeedSetpointInfo().getMaximum();
				
				// Now calculate pivot amount
				// - Strength of pivot (nPivSpeed) based on  X input
				// - Blending of pivot vs drive (fPivScale) based on Y input
				float fPivYLimit = 250.0f;
				int nPivSpeed = (int) th;
				// if th beyond pivlimit scale in 
				float fPivScale = (float) ((Math.abs(x)>fPivYLimit)? 0.0 : (1.0 - Math.abs(x)/fPivYLimit));

				// Calculate final mix of Drive and Pivot
				/* Set the target speeds in meters per second */
				spd_left = (float) ((1.0-fPivScale)*spd_left + fPivScale*( nPivSpeed));
				spd_right = (float) ((1.0-fPivScale)*spd_right + fPivScale*(-nPivSpeed));
			}
		}

		// Calculate final mix of Drive and Pivot
		// Set the target speeds in wheel rotation command units -1000, 1000 and if indoor mode div by ten
		if(robot.getDiffDrive().isIndoor()) {
			robot.getLeftSpeedSetpointInfo().setTarget(spd_left/10);
			robot.getRightSpeedSetpointInfo().setTarget(spd_left/10);
		} else {
			robot.getLeftSpeedSetpointInfo().setTarget(spd_left);
			robot.getRightSpeedSetpointInfo().setTarget(spd_left);
		}
		if( DEBUG )
			System.out.println("Linear x:"+x+" angular z:"+th+" Motor L:"+robot.getLeftSpeedSetpointInfo().getTarget()+
								" R:"+robot.getRightSpeedSetpointInfo().getTarget());
		/* Convert speeds to ticks per frame */
		//robot.getDiffDrive().getLeftWheel().getSpeedsetPointInfo().SpeedToTicks(robot.getLeftMotorPIDController(), robot.getDiffDrive().getLeftWheel());
		//robot.getDiffDrive().getRightWheel().getSpeedsetPointInfo().SpeedToTicks(robot.getRightMotorPIDController(), robot.getDiffDrive().getRightWheel());
		//leftWheel.TargetTicksPerFrame = SpeedToTicks((float) leftWheel.TargetSpeed);
		//rightWheel.TargetTicksPerFrame = SpeedToTicks((float) rightWheel.TargetSpeed);
		/* Read the encoders */
		//leftWheel.Encoder = 0;//encoders.YAxisGetCount();
		//rightWheel.Encoder = 0;//encoders.XAxisGetCount();

		/* Record the time that the readings were taken */
		//robot.getDiffDrive().odomInfo.lastOdomTime = System.currentTimeMillis();
		//odomInfo.encoderStamp = nh.now;

		/* Compute PID update for each motor */
		robot.getLeftMotorPIDController().Compute(robot.getLeftSpeedSetpointInfo());
		robot.getRightMotorPIDController().Compute(robot.getRightSpeedSetpointInfo());

		/* Set the motor speeds accordingly */
		//if( DEBUG )
		//	System.out.println("Motor:"+leftWheel.TargetSpeed+" "+rightWheel.TargetSpeed);
		// call to motor controller to update speed using absolute terms
		//motorControl.updateSpeed(slot1, channel1, (int)leftWheel.TargetSpeed, slot2, channel2, (int)rightWheel.TargetSpeed);
		return new int[]{(int) robot.getLeftSpeedSetpointInfo().getTarget(),
						 (int) robot.getRightSpeedSetpointInfo().getTarget()};
	}
	

	
	/**
	 * Move the buffered values into the publishing message to send absolute vals to motor and peripheral control.
	 * We will be sending [<slot> <channel> <value>]
	 * We use a Int32MultiArray type of 2 dimensions, row of each value times column of all values.
 	 * Our setup is 10 possible slots for either motor or peripheral controllers
 	 * 10 possible channels per controller
 	 * 1 value per slot and channel.
	 * @param connectedNode Our node transceiver info
	 * @param valBuf The linear array of values to send, which we will be transposing to our MultiAray
	 * @param label1 The label for dimension 1 of the array
	 * @param label2 The label for dimension 2 of the array
	 * @return The multi array we create
	 */
	private std_msgs.Int32MultiArray setupPub(ConnectedNode connectedNode, ArrayList<Integer> valBuf, String label1, String label2) {
		return RosArrayUtilities.setupInt32Array(connectedNode, valBuf, label1, label2);
	}
	

	/*
	 // Create Roll Pitch Yaw Angles from Quaternions 
	double yy = quat.y() * quat.y(); // 2 Uses below
	double roll = atan2(2 * (quat.w() * quat.x() + quat.y() * quat.z()), 1 - 2*(quat.x() * quat.x() + yy));
	double pitch = asin(2 * quat.w() * quat.y() - quat.x() * quat.z());
	double yaw = atan2(2 * (quat.w() * quat.z() + quat.x() * quat.y()), 1 - 2*(yy+quat.z() * quat.z()));

	//Convert Radians to Degrees 
	float rollDeg  = 57.2958 * roll;
	float pitchDeg = 57.2958 * pitch;
	float yawDeg   = 57.2958 * yaw;
	 */
}
