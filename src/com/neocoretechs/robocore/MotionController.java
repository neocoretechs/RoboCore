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
	public static float WHEELBASE = 1.0f;
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
	boolean holdBearing = false; // hold steering to current bearing

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
		//
		subsrange.addMessageListener(new MessageListener<sensor_msgs.Joy>() {
			@Override
			public void onNewMessage(sensor_msgs.Joy message) {
				float speedL, speedR;
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
						System.out.println("Stick absolute deg set:"+Setpoint);	
					} // If we are pressing triangle, leave setpoint alone as initial bearing from IMU
			    } else {
			    	holdBearing = false;
			    	Setpoint = offsetDegrees; // otherwise its the heading minus the joystick offset from 0
			    }
	
				Input = (float) eulers[0]; //eulers[0] is yaw angle from IMU
				Compute();
	
				// Output has the positive or negative difference in degrees (left or right offset)
				// between IMU heading and joystick, we have to calc the scaling factors for wheel speed then apply
				// We determine the radius of the turn potential by the forward speed desired (joystick Y)
				// For instance if at 0 we can spin at radius .5 of wheelbase if necessary, while at speed 500 out of 1000
				// we can turn at radius 1, which is 90 degrees with inner wheel stationary, and at 1000 out of 1000
				// we turn inner radius 1 and outer radius 2 robot widths, a gradual arc.
				// degrees/180 * PI * (r + WHEELBASE) = outer wheel arc length
				// degrees/180 * PI * r = inner wheel arc length
				// we compare the arc lengths as a ratio.
				// We use the speed in the range 1,0,-1 as a scale factor to add to wheelbase, so at max speed
				// we extend the radius by one robot length; joystick y + wheelbase making inner radius 1 and outer 2.
				// This assumes that at half speed we can still make a 90 degree turn.
				// We use value of IMU vs desired course to turn left or right via
				// a plus or minus value from the Compute method set in the variable called Output.
				float radius = Math.abs(axes[2]);
				float arcin = (float) ((radius/180.0) * Math.PI * radius);
				float arcout = (float) ((radius/180.0) * Math.PI * (radius + WHEELBASE));
				// speed may be plus or minus, this determines a left or right turn as the quantity is added to desired speed
				// of wheels left and right
				// axes[2] is y, value is -1 to 0 to 1 for some reason forward is negative on the stick
				speedR = -axes[2] * 1000.0f;
				speedL = speedR;
				// Output is difference in degrees between setpoint and input
				if( Output < 0.0f )
					speedL *= (arcin/arcout);
				else
					if( Output > 0.0f )
						speedR *= (arcin/arcout);
				System.out.printf("Stick deg=%f|Offset deg=%f|IMU=%f|arcin=%f|arcout=%f|Output=%f|speedL=%f|speedR=%f|Hold=%b\n",stickDegrees,offsetDegrees,eulers[0],arcin,arcout,Output,speedL,speedR,holdBearing);
				// set it up to send
				ArrayList<Integer> speedVals = new ArrayList<Integer>(2);
				speedVals.add((int)speedL);
				speedVals.add((int)speedR);
				velpub.publish(setupPub(connectedNode, speedVals));		
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
				//SetTunings(5.55f, 1.0f, 0.5f); // 5.55 scales a max +-180 degree difference to the 0 1000,0 -1000 scale
				SetTunings(1.0f, 1.0f, 0.5f);
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
	
	static float steadyError, previousError;
	static long baset = System.currentTimeMillis();
	static long samplet = 100; // millis sample time
	static float kp = 0.0f;
	static float ki = 1.0f;
	static float kd = 0.0f;
	static float pidOutput = 0;
	// pid controller
	static void updatePID(float current, float target)
	{
		float P; // proportional error
		float I; // integral error
		float D; // derivative error
		// determine time interval
		long difft = System.currentTimeMillis() - baset;
		if( difft < samplet )
			return;
		baset = System.currentTimeMillis();
		float dt = (float)difft;
	    // calculate difference between actual and goal values
	    float pidError = target - current;
	    if (pidError < -270) pidError += 360;
	    if (pidError >  270) pidError -= 360;
	 
	    // accumulates error over time
	    steadyError += pidError*dt;
	 
	    // integrator windup compensation (saturates to actuator's output)
	    if (steadyError < -10.0) steadyError = -10.0f;
	    if (steadyError > 10.0) steadyError = 10.0f;
	 
	    P = kp*pidError; // proportional error
	    I = ki*steadyError; // integral error
	    D = kd*(pidError - previousError)/dt;  // derivative error
	 
	    // save current error
	    previousError = pidError;
	 
	    pidOutput = P + I + D;  // calculate the PID output
	    //pidOutput /= 100.0;  // scale it down to get it within the range of the actuator - probably needs tuning
	    //if(pidOutput < -0.1) pidOutput = -0.1f;
	    //if(pidOutput > 0.1) pidOutput = 0.1f;
	 
	    System.out.printf("P = %f | I = %f | D = %f | Output = %f | pidErr = %f | SteadyErr = %f\n",P,I,D,pidOutput,pidError,steadyError);
	    // publish to the absolute motor control channel
	    
	}
	// working variables
	static long lastTime = System.currentTimeMillis();
	static float Input, Output, Setpoint;
	static float ITerm, lastInput;
	//float kp, ki, kd;
	static int SampleTime = 100; //.1 sec
	static float outMin, outMax;
	static boolean inAuto = false;
	static void Compute()
	{
	   //if(!inAuto) return;
	   //long now = System.currentTimeMillis();
	   //long timeChange = (now - lastTime);
	   //if(timeChange >= SampleTime) {
	      // Compute all the working error variables
	      float error = Setpoint - Input;
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
	      // To change tuning parameters at runtime:
	      // Instead of having the Ki live outside the integral, we bring it inside.
	      // we take the error and multiply it by whatever the Ki is at that time. 
	      // We then store the sum of THAT. When the Ki changes, there no bump because 
	      // all the old Kis are already in the bank so to speak. 
	      // We get a smooth transfer with no additional math operations
	      //ITerm += (ki * error);
	      // clamp the I term to prevent reset windup
	      // If all we did was clamp the output, the Integral term would go back to growing and growing. 
	      // Though the output would look nice during the step up, wed see that telltale lag on the step down
	      //if(ITerm > outMax) 
	    	//  ITerm = outMax;
	      //else 
	    	//  if(ITerm < outMin) 
	    	//	  ITerm = outMin;
	      //float dInput = (Input - lastInput);
	 
	      // Compute PID Output
	      Output = kp * error ;//+ ITerm - kd * dInput;
	      //In addition to clamping the I-Term, we clamp the Output value so that it stays where wed expect it.
	      /*
	      if(Output > outMax) 
	    	  Output = outMax;
	      else 
	    	  if(Output < outMin) 
	    		  Output = outMin;
	 	  */
	      // Remember some variables for next time
	      lastInput = Input;
	      //lastTime = now;
	      //error is positive if current_heading > bearing (compass direction)
	      //positive bias acts to reduce left motor speed, so bear left

	      //System.out.printf("P = %f | I = %f | D = %f | Output = %f | pidErr = %f\n",error,ITerm,dInput,Output,error);
	      System.out.printf("Output L = %f | Output R = %f | Err = %f\n",-Output, Output,error);
	   //}
	}
	 
	static void SetTunings(float Kp, float Ki, float Kd)
	{
	  float SampleTimeInSec = ((float)SampleTime)/1000.0f;
	  kp = Kp;
	  ki = Ki * SampleTimeInSec;
	  kd = Kd / SampleTimeInSec;
	}
	 
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
	 * Set output limits to clamp I term and output
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
}
