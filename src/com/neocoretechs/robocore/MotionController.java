package com.neocoretechs.robocore;

//import org.apache.commons.logging.Log;
import java.io.File;
import java.net.InetSocketAddress;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;
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


/**
 * We are fusing data from the IMU, any joystick input, and other autonomous controls to affect
 * propulsion and issue status updates based on IMU data edge conditions that may indicate problems.
 * Published on absolute/cmd_vel are the values from 0-1000 that represent actual power to each differential drive wheel.
 * Subscribers to absolute/cmd_vel include the process that manages serial data to the motor controller.
 * Publishers on robocore/status provide status updates relating to IMU extremes that are picked up by the VoxHumana voice synth or others.
 * sensor_msgs/MagneticField
 * sensor_msgs/Temperature
 * sensor_msgs/Imu
 * UpperFront/sensor_msgs/Range
 * LowerFront/sensor_msgs/Range
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
 * @author jg Copyright (C) NeoCoreTechs 2017,2018
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
	
	public static boolean isShock = false;
	public static boolean isOverShock = false;
	// If we call the wheelbase 1000, it implies that at maximum speed of 1000 it take 2 robot radii to turn 90 degrees
	// This is because the radius of the inner wheel is from speed 0 to speed max, and the outer is from speed 0 to speed max plus wheelbase.
	// Correspondingly at speed 1 the arc describing the inner wheel is 1 and the inner wheel turns at 1, or in place 
	// and the arc describing the outer wheel has a radius equivalent to the wheelbase and the 
	// outer wheel turns at the robot wheelbase through 90 degrees.
	public static float WHEELBASE = 1000.0f;
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
	public static int TEMPERATURE_THRESHOLD = 40; // C 104f
	public static boolean isTemperature = false;
	public static boolean isOverTemp = false;
	public static boolean isMoving = false;
	public static boolean isNav = false;
	public static boolean isRangeUpperFront = false;
	public static boolean isRangeLowerFront = false;
	public static boolean isVision = false; // machine vision recognition event
	
	boolean hasData = false; // have we received any feedback from callback?
	boolean init = true;
	static boolean holdBearing = false; // hold steering to current bearing

	static float speedL, speedR;
	static float pidOutput = 0;
	static float kp = 0.9f;
	static float ki = .85f;
	static float kd = 3.0f;
	static long lastTime = System.currentTimeMillis();
	static float Input, Output, Setpoint, error;
	static float ITerm, DTerm, lastInput, lastError;
	static int SampleTime = 100; //.1 sec
	static float outMin, outMax;
	static boolean inAuto = false;
	static boolean outputNeg = false; // used to prevent integral windup by resetting ITerm when crossing track
	
	Object rngMutex1 = new Object();
	Object rngMutex2 = new Object();
	Object navMutex = new Object();
	Object magMutex = new Object();
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
	    try {
			awaitStart.await();
		    if( DEBUG )
		    	System.out.println("Pub/subs registered..");
		} catch (InterruptedException e) {}
	    
	}
	
	public MotionController(String[] args) {
		CommandLineLoader cl = new CommandLineLoader(Arrays.asList(args));
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, cl.build());
	    if( DEBUG ) {
	    	System.out.println("Bringing up MotionControl with args:"+args[0]+" "+args[1]+" "+args[2]);
	    }
	    try {
			awaitStart.await();
		    if( DEBUG )
		    	System.out.println("Pub/subs registered..");
		} catch (InterruptedException e) {}
	}
	
	public MotionController() { }
	
	/**
	 * Create NodeConfiguration 
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
		//final Log log = connectedNode.getLog();
		Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
		if( remaps.containsKey("__wheelbase") )
			WHEELBASE = Float.parseFloat(remaps.get("__wheelbase"));
		
		//final Publisher<geometry_msgs.Twist> mopub = connectedNode.newPublisher("cmd_vel", geometry_msgs.Twist._TYPE);
		// statpub has status alerts that may come from sensors.
		
		final Publisher<diagnostic_msgs.DiagnosticStatus> statpub =
				connectedNode.newPublisher("robocore/status", diagnostic_msgs.DiagnosticStatus._TYPE);
		final Publisher<std_msgs.Int32MultiArray> velpub =
				connectedNode.newPublisher("absolute/cmd_vel", std_msgs.Int32MultiArray._TYPE);
		
		//final geometry_msgs.Twist twistmsg = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Twist._TYPE);
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
		//
		// Joystick data will have array of axis and buttons, axis[0] and axis[2] are left stick x,y axis[1] and axis[3] are right.
		// The code calculates the theoretical speed for each wheel in the 0-1000 scale based on target point vs IMU yaw angle.
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
		//
		subsrange.addMessageListener(new MessageListener<sensor_msgs.Joy>() {
			@Override
			public void onNewMessage(sensor_msgs.Joy message) {
				float[] axes = message.getAxes();
				int[] buttons = message.getButtons();
				// check for emergency stop, on X or A or green or lower button
				if( buttons[6] != 0 ) {
					System.out.println("**CANCELLED "+axes[2]);
					return;
				}
				// If the button square or circle is depressed, rotate in place at stick position Y speed
				if( buttons[7] != 0 ) { // left pivot
					speedR = -axes[2] * 1000.0f;
					speedL = -speedR;
					// set it up to send
					ArrayList<Integer> speedVals = new ArrayList<Integer>(2);
					speedVals.add((int)speedL);
					speedVals.add((int)speedR);
					velpub.publish(setupPub(connectedNode, speedVals));
					System.out.printf("Stick Left pivot speedL=%f|speedR=%f\n",speedL,speedR);
					return;
				} else {
					if( buttons[5] != 0 ) { // right
						speedL = -axes[2] * 1000.0f;
						speedR = -speedL;
						// set it up to send
						ArrayList<Integer> speedVals = new ArrayList<Integer>(2);
						speedVals.add((int)speedL);
						speedVals.add((int)speedR);
						velpub.publish(setupPub(connectedNode, speedVals));
						System.out.printf("Stick Right pivot speedL=%f|speedR=%f\n",speedL,speedR);
						return;
					}
				}
	
				float radians = (float) Math.atan2(axes[2], axes[0]);
				float stickDegrees = (float) (radians * (180.0 / Math.PI)); // convert to degrees
				// horizontal axis is 0 to -180 degrees, we want 0 at the top
				stickDegrees += 90;
				stickDegrees = (stickDegrees > 180.0 ? stickDegrees -= 90.0 : -stickDegrees);
				// we have absolute stickdegrees, offset the current IMU by that quantity to get new desired course
				float offsetDegrees = (float) (eulers[0] - stickDegrees);
			    // reduce the angle  
			    offsetDegrees =  offsetDegrees % 360; 
			    // force it to be the positive remainder, so that 0 <= angle < 360  
			    offsetDegrees = (offsetDegrees + 360) % 360;  
			    // force into the minimum absolute value residue class, so that -180 < angle <= 180  
			    if (offsetDegrees > 180)  
			          offsetDegrees -= 360;
				//if( DEBUG )
				//	System.out.println("Axes:"+axes[0]+","+axes[2]+" deg:"+bearingDegrees+" rad:"+radians);
				// If we are pressing the triangle, dont update the setpoint with IMU yaw, this has the effect of locking
				// the course onto the absolute track relative to the way the robot is pointing offset by stick position
				// rather than a continuous update of stick offset added to current orientation. so if you want the robot to track
				// straight forward on its current orientation, hold triangle and push stick forward or back.
			    if( buttons[4] != 0 ) {   
					// is it initial button press? then log current bearing of IMU to hold to
					if( !holdBearing ) {
						holdBearing = true;
						Setpoint = (float) eulers[0];
						Output = 0.0f;
						ITerm = 0.0f;
						DTerm = 0.0f;
						System.out.println("Stick absolute deg set:"+Setpoint);	
					} // If we are pressing triangle, leave setpoint alone as initial bearing from IMU
			    } else {
			    	holdBearing = false;
			    	Setpoint = offsetDegrees; // otherwise its the heading minus the joystick offset from 0
			    }
	
				Input = (float) eulers[0]; //eulers[0] is yaw angle from IMU
				Compute();
				
				// Speed may be plus or minus, this determines a left or right turn as the quantity is added to desired speed
				// of wheels left and right. The speed for each wheel is modified by desired turn radius, which is based on speed,
				// or on the computed derivation from desired course if a straight line course is set.
				// axes[2] is y, value is -1 to 0 to 1, and for some reason forward is negative on the stick
				// scale it from -1 to 0 to 1 to -1000 to 0 to 1000, which is our speed control range 
				speedR = -axes[2] * 1000.0f;
				speedL = speedR;
				// Output has the positive or negative difference in degrees (left or right offset)
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
				// a plus or minus value from the Compute method set in the variable called Output.
				float radius = Math.abs(axes[2]) * 1000.0f;
				float arcin, arcout;
				//
				// If holding an inertially guided course, check for the tolerance of error proportion
				// from setpoint to current heading, if within that tolerance, use the pid values of the
				// currently computed course deviation plus the scaled speed dependent radius from stick value to
				// alter the motor speed to reduce cross track error and become critically damped.
				//
				if( holdBearing ) {
					System.out.printf("Inertial Setpoint=%f | Hold=%b ", Setpoint, holdBearing);
					// In auto
					if( Math.abs(Output) <= 30.0 ) {
						if( Output < 0.0f ) { // increase right wheel power goal
							//rightPid(axes);
							rightAngle(radius); // decrease left wheel power
						} else {
							if( Output > 0.0f ) { // increase left wheel power goal
								//leftPid(axes);
								leftAngle(radius); // decrease right wheel power
							} else {
								// we are critically damped, set integral to 0
								ITerm = 0;
							}
						}
						System.out.printf("<=30 degrees Speed=%f|IMU=%f|speedL=%f|speedR=%f|Hold=%b\n",radius,eulers[0],speedL,speedR,holdBearing);
					} else {
						// Exceeded tolerance, proceed to geometric solution, reset integral windup
						ITerm = 0;
						arcin = (float) (Math.abs(Output/360.0) * (2.0 * Math.PI) * radius);
						arcout = (float) (Math.abs(Output/360.0) * (2.0 * Math.PI) * (radius + WHEELBASE));
						// Output is difference in degrees between setpoint and input
						if( Output < 0.0f )
							speedL *= (arcin/arcout);
						else
							if( Output > 0.0f )
								speedR *= (arcin/arcout);
						System.out.printf(">30 degrees Speed=%f|IMU=%f|arcin=%f|arcout=%f|speedL=%f|speedR=%f|Hold=%b\n",radius,eulers[0],arcin,arcout,speedL,speedR,holdBearing);
					}
				} else {
					// manual steering mode, use tight radii and a human integrator
					arcin = (float) (Math.abs(Output/360.0) * (2.0 * Math.PI) * radius);
					arcout = (float) (Math.abs(Output/360.0) * (2.0 * Math.PI) * (radius + WHEELBASE));
					// Output is difference in degrees between setpoint and input
					if( Output < 0.0f )
						speedL *= (arcin/arcout);
					else
						if( Output > 0.0f )
							speedR *= (arcin/arcout);
					System.out.printf("Stick deg=%f|Offset deg=%f|IMU=%f|arcin=%f|arcout=%f|speedL=%f|speedR=%f|Hold=%b\n",stickDegrees,offsetDegrees,eulers[0],arcin,arcout,speedL,speedR,holdBearing);
				}
	
				System.out.printf("Output = %f | DTerm = %f | ITerm = %f | Err = %f | PID=%f | Hold=%b\n",Output, DTerm, ITerm, error, pidOutput, holdBearing);
				// set it up to send
				ArrayList<Integer> speedVals = new ArrayList<Integer>(2);
				speedVals.add((int)speedL);
				speedVals.add((int)speedR);
				velpub.publish(setupPub(connectedNode, speedVals));		
			}
			/**
			 * Increase power of right wheel based on PID results
			 * If we hit max we are going to decrease power to left wheel by splitting the difference
			 * of leftover speed on right applied to left wheel decrease. whether these are equivalent in reality is
			 * left to evaluation.
			 * @param axes
			 */
			private void rightPid(float[] axes) {
				if( !outputNeg ) { // prevent integral windup by checking when track is crossed via sign of course diff changing
					ITerm = 0;
				}
				outputNeg = true;
				// goal is to increase power of right wheel
				// We scale it by speed with the assumption that at full speed,
				// the values from PID represent the ones necessary to work at top speed of 1000.
				speedR += ( Math.abs(pidOutput) * (Math.abs(axes[2])*3.7) );
				// if we cap at max, use the difference to retard opposite
				if( speedR > 1000.0f ) {
					float speeDif = speedR - 1000.0f;
					speedR = 1000.0f;
					// left is decreased by the amount right went over max
					speedL -= speeDif;
					if( speedL < 0.0 ) speedL = 0.0f;
					// and the robot is pivoting at max speed if everything was capped
				}
			}
			/**
			 * Increase power of left wheel base don PID results. If we max out we are going to apply the leftover
			 * power over maximum to decreasing the right wheel speed.
			 * @param axes
			 */
			private void leftPid(float[] axes) {
				if( outputNeg ) {
					ITerm = 0;
				}
				outputNeg = false;
				// goal is net increase of left wheel power
				// We scale it by speed with the assumption that at full speed,
				// the values from PID represent the ones necessary to work at top speed of 1000.
				speedL +=( Math.abs(pidOutput) * (Math.abs(axes[2])*3.7) );
				if( speedL > 1000.0f ) {
					float speeDif = speedL - 1000.0f;
					speedL = 1000.0f;
					speedR -= speeDif;
					if( speedR < 0.0) speedR = 0.0f;
				}
			}
			/**
			 * Apply a solution in triangles as we do with solution in arcs. This presupposes we are within tolerance such that
			 * the long leg is not much beyond the 'WHEELBASE' when we finally compute it, 
			 * WHEELBASE is not really the physical, actual wheelbase, but a scaling value to get us into our
			 * 0-1000 speed range.
			 */
			private void rightAngle(float radius) {
				// maintain our PID as well
				if( !outputNeg ) { // prevent integral windup by checking when track is crossed via sign of course diff changing
					ITerm = 0;
				}
				outputNeg = true;
				// compute chord of course deviation, this is short leg of triangle.
				// 'radius' is our scaled speed, it does not point directly forward but is used for arc cord and so it is an angle.
				// might need to scale
				// chord is 2Rsin(theta/2) ( we convert to radians first)
				double chord = 2 * (radius*1.1) * Math.sin((Math.abs(Output/360.0) * (2.0 * Math.PI))/2);
				// make the base of the triangle radius , the do a simple pythagorean deal and take the 
				// square root of sum of square of base and leg
				double hypot = Math.sqrt((chord*chord) + (radius*radius));
				// decrease the power on the opposite side by ratio of base to hypotenuse, since one wheel needs to 
				// travel the length of base and the other needs to travel length of hypotenuse
				System.out.printf(" RIGHTANGLE=%f|chord=%f|hypot=%f|radius/hypot=%f|Hold=%b\n",radius,chord,hypot,(radius/hypot),holdBearing);
				speedL *= (radius/hypot);
			}
			
			private void leftAngle(float radius) {
				// preserve PID integral
				if( outputNeg ) {
					ITerm = 0;
				}
				outputNeg = false;
				// compute chord of course deviation, this is short leg of triangle.
				// 'radius' is our scaled speed, it does not point directly forward but is used for arc cord and so it is an angle.
				// might need to scale
				// chord is 2Rsin(theta/2)
				double chord = 2 * (radius*1.1) * Math.sin((Math.abs(Output/360.0) * (2.0 * Math.PI))/2);
				// make the base of the triangle radius , the do a simple pythagorean deal and take the 
				// square root of sum of square of base and leg
				double hypot = Math.sqrt((chord*chord) + (radius*radius));
				// decrease the power on the opposite side by ratio of base to hypotenuse, since one wheel needs to 
				// travel the length of base and the other needs to travel length of hypotenuse
				System.out.printf(" LEFTANGLE=%f|chord=%f|hypot=%f|radius/hypot=%f|Hold=%b\n",radius,chord,hypot,(radius/hypot),holdBearing);
				speedR *= (radius/hypot);
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
					System.out.println("Nav:Orientation X:"+orientation.getX()+" Y:"+orientation.getY()+" Z:"+orientation.getZ()+" W:"+orientation.getW());
					System.out.println("Nav:Eulers "+eulers[0]+" "+eulers[1]+" "+eulers[2]);
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
							System.out.printf("Nav:Angular X:%f | Angular Y:%f | Angular Z:%f",angular.getX(),angular.getY(),angular.getZ());
							System.out.printf("Nav:Linear X:%f | Linear Y:%f | Linear Z:%f",linear.getX(),linear.getY(),linear.getZ());
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
		//
		// Begin publishing loop
		//
		connectedNode.executeCancellableLoop(new CancellableLoop() {
			private int sequenceNumber;

			@Override
			protected void setup() {
				sequenceNumber = 0;
				Setpoint = 0.0f; // 0 degrees yaw
				// default kp,ki,kd;
				//SetTunings(5.55f, 1.0f, 0.5f); // 5.55 scales a max +-180 degree difference to the 0 1000,0 -1000 scale
				SetOutputLimits(0.0f, 1000.0f);
				SetMode(true);
			}

			@Override
			protected void loop() throws InterruptedException {
		
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
	 * Move the buffered values into the publishing message to send absolute vals to motor control
	 * @param connectedNode
	 * @return
	 */
	private std_msgs.Int32MultiArray setupPub(ConnectedNode connectedNode, ArrayList<Integer> valBuf) {
		std_msgs.Int32MultiArray val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Int32MultiArray._TYPE);
		std_msgs.MultiArrayLayout vlayout = new std_msgs.MultiArrayLayout();
		List<std_msgs.MultiArrayDimension> vdim = new ArrayList<std_msgs.MultiArrayDimension>();
		std_msgs.MultiArrayDimension vdim1 = new std_msgs.MultiArrayDimension();
		std_msgs.MultiArrayDimension vdim2 = new std_msgs.MultiArrayDimension();
		vdim1.setLabel("Motor Channel");
		vdim2.setLabel("Motor Channel value");
		vdim1.setSize(2);
		vdim2.setSize(valBuf.size()/2);
		vdim1.setStride(valBuf.size());
		vdim2.setStride(2);
		vdim.add(vdim1);
		vdim.add(vdim2);
		vlayout.setDim(vdim);
		val.setLayout(vlayout);
		int[] vali = new int[valBuf.size()];
		int i = 0;
		for( Integer inv : valBuf)
			vali[i++] = inv;
		val.setData(vali);
		return val;
	}
	/**
	 * Calculate the PID values, Output contains the proportion times scalar
	 * DTerm is the derivative, ITerm is the integral of error variable.
	 * pidOutput is output.
	 */
	static void Compute()
	{
	   //if(!inAuto) return;
	   //long now = System.currentTimeMillis();
	   //long timeChange = (now - lastTime);
	   //if(timeChange >= SampleTime) {
	      lastError = error;
	      // Compute all the working error variables
	      error = Setpoint - Input;
	      // This is specific to pid control of heading
	      // reduce the angle  
	      error =  error % 360; 
	      // force it to be the positive remainder, so that 0 <= angle < 360  
	      error = (error + 360) % 360;  
	      // force into the minimum absolute value residue class, so that -180 < angle <= 180  
	      if (error > 180)  
	          error -= 360;  
		  //if (error < -270) error += 360;
		  //if (error >  270) error -= 360;  

	      //float dInput = (Input - lastInput);
	      // Compute PID Output, proportion
	      Output = kp * error ;//+ ITerm - kd * dInput;
	      // derivative
	      DTerm = kd * (error - lastError);
	      // integral
	      ITerm += error; // integrate the error
	      ITerm = ki * ITerm; // then scale it, this limits windup
	      //ITerm += (ki * error);
	      // clamp the I term to prevent reset windup
	      if( ITerm > 0 ) {
	    	  if(ITerm > outMax) 
	    		  ITerm = outMax;
	    	  else 
	    		  if(ITerm < outMin) 
	    			  ITerm = outMin;
	      } else {
	    	  if(-ITerm > outMax )
	    		  ITerm = -outMax;
	    	  else
	    		  if(-ITerm < outMin)
	    			  ITerm = -outMin;
	      }
	      lastInput = Input;
	      //lastTime = now;
	      //error is positive if current_heading > bearing (compass direction)
	      // To Use the PD form, set ki to 0.0 default, thus eliminating integrator
	      pidOutput = Output + ITerm + DTerm;
	      //System.out.printf("P = %f | I = %f | D = %f | Output = %f | pidErr = %f\n",error,ITerm,dInput,Output,error);
	      //System.out.printf("Output = %f | DTerm = %f | ITerm = %f | Err = %f | PID=%f | Auto = %b\n",Output, DTerm, ITerm, error, pidOutput, holdBearing);
	   //}
	}
	/**
	 * Currently we are using a non time based control, this is for reference.
	 * @param Kp
	 * @param Ki
	 * @param Kd
	 */
	static void SetTunings(float Kp, float Ki, float Kd)
	{
	  float SampleTimeInSec = ((float)SampleTime)/1000.0f;
	  kp = Kp;
	  ki = Ki * SampleTimeInSec;
	  kd = Kd / SampleTimeInSec;
	}
	 /**
	  * If we used a time based control.
	  * @param NewSampleTime
	  */
	static void SetSampleTime(int NewSampleTime)
	{
	   if( NewSampleTime > 0) {
	      float ratio  = (float)NewSampleTime / (float)SampleTime;
	      ki *= ratio;
	      kd /= ratio;
	      SampleTime = NewSampleTime;
	   }
	}
	/**
	 * Set output limits to clamp Iterm and output
	 * @param Min
	 * @param Max
	 */
	static void SetOutputLimits(float Min, float Max)
	{
	   if(Min > Max) return;
	   outMin = Min;
	   outMax = Max;
	    
	   if(Output > outMax) 
		   Output = outMax;
	   else 
		   if(Output < outMin) 
			   Output = outMin;
	 
	   if(ITerm > outMax) 
		   ITerm= outMax;
	   else 
		   if(ITerm < outMin) 
			   ITerm= outMin;
	}
	 
	static void SetMode(boolean isAuto)
	{
	    if(isAuto && !inAuto)
	    {  
	    	/*we just went from manual to auto*/
	        Initialize();
	    }
	    inAuto = isAuto;
	}
	/**
	 * Init lastInput (last IMU), ITerm integral , and check for integral windup
	 */
	static void Initialize()
	{
	   lastInput = Input;
	   ITerm = Output;
	   if(ITerm > outMax) 
		   ITerm = outMax;
	   else 
		   if(ITerm < outMin) 
			   ITerm = outMin;
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
