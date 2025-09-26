package com.neocoretechs.robocore.propulsion;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.CountDownLatch;
import java.util.stream.Collectors;

import org.ros.concurrent.CancellableLoop;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.internal.node.topic.PublisherIdentifier;
import org.ros.internal.node.topic.SubscriberIdentifier;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.PublisherListener;
import org.ros.node.topic.Subscriber;
import org.ros.node.topic.SubscriberListener;

import com.neocoretechs.robocore.RosArrayUtilities;
import com.neocoretechs.robocore.PID.IMUSetpointInfo;
import com.neocoretechs.robocore.PID.MotionPIDController;
import com.neocoretechs.robocore.config.DeviceEntry;
import com.neocoretechs.robocore.config.Robot;
import com.neocoretechs.robocore.config.RobotInterface;
import com.neocoretechs.robocore.config.TypedWrapper;
import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;
import com.neocoretechs.robocore.marlinspike.MarlinspikeManager;
import com.neocoretechs.robocore.marlinspike.NodeDeviceDemuxer;
import com.neocoretechs.robocore.marlinspike.PublishDiagnosticResponse;

import org.json.JSONObject;

import sensor_msgs.Joy;
import std_msgs.Int32MultiArray;

/**
 * We are fusing data from the IMU, any joystick input, and other autonomous controls to send down to the Marlinspike controllers.
 * Subscribing to:
 * sensor_msgs/Joy (joystick information boxed as an array of buttons and axis values common to standard controllers):
 * * NYCO CORE CONTROLLER:
 * Identifies as : USB Joystick
 *Axis x		Left stick (left/right)
 *Axis y		Left stick (up/down)
 *Axis rx		N/A	 
 *Axis ry		N/A	 
 *Axis z	 	Right stick (left/right)
 *Axis rz		Right stick (up/down)
 *Button 0	x  - Thumb 2
 *Button 1	circle - Thumb
 *Button 2	square - Top
 *Button 3	triangle - Trigger
 *Button 4	Left Bumper	- Top2
 *Button 5	Right Bumper - Pinkie
 *Button 6	Select - Base3
 *Button 7	Start - Base4
 *Button 8	Left Stick Press - Base5
 *Button 9	Right Stick Press - Base6
 *
 * AFTERGLOW CONTROLLER:
 * Identifies as : Generic X-Box Pad (right, even though its a PS3!)
 *Axis x		Left stick (left/right)
 *Axis y		Left stick (up/down)
 *Axis z	 	Left trigger
 *Axis rx(rx)	Right stick (left/right)	 
 *Axis ry(ry)	Right stick (up/down) 
 *Axis rz		Right trigger
 *Button B(B) - circle
 *Button X(X) - square
 *Button A(A) - X
 *Button Y(Y) - triangle
 *Button Left Thumb - left bumper
 *Button Right Thumb - right bumper
 *Button Left Thumb 3 - left stick press
 *Button Right Thumb 3 - right stick press
 *Button Select - Select
 *Button Mode - Mode
 *Button Unknown - Home
 *
 * LOGITECH, INC F310 GAMEPAD
 * Identifies as : Generic X-box pad (Controls are the same as AfterGlow) or Logitech Gamepad F310
 *Axis x		Left stick (left/right)
 *Axis y		Left stick (up/down)
 *Axis z	 	Left trigger
 *Axis rx(rx)	Right stick (left/right)	 
 *Axis ry(ry)	Right stick (up/down) 
 *Axis rz		Right trigger
 *Button B(B) - circle
 *Button X(X) - square
 *Button A(A) - X
 *Button Y(Y) - triangle
 *Button Left Thumb - left bumper
 *Button Right Thumb - right bumper
 *Button Left Thumb 3 - left stick press
 *Button Right Thumb 3 - right stick press
 *Button Select - Select
 *Button Mode - Mode
 *Button Unknown - Home
 *
 * sensor_msgs/MagneticField (3 axis magnetic flux information most likely from IMU)
 * sensor_msgs/Temperature (most likely from IMU)
 * sensor_msgs/Imu (absolutely from IMU, contains heading information roll/pitch/yaw in Euler and Quaternion, pre-fused)
 *
 * Publishing to:
 * All topics enumerated in MegaPubs typeNames which correspond to types that may appear in configuration by LUN.k
 * robocore/status (status update channel to receive information on alerts, etc, derived from IMU and other sensors)
 * 
 * Published are the values from 0-1000 that represent actual power to each differential drive wheel.
 * Subscribers include the process that manages serial data to the motor controller, possibly on a separate computer.
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
	private static boolean IMUDEBUG = false;
	private static boolean DEBUGBEARING = true;
	//private String host;
	//private InetSocketAddress master;
	private CountDownLatch awaitStart = new CountDownLatch(1);

	//private String serveNode = null;
	float rangetop, rangebot;
	double pressure, temperature;
	static class EulerTime {
		double eulers[] = new double[]{0.0,0.0,0.0}; // set from loop
		long eulerTime = 0L;
		public String toString() {
			return String.format("Eulers: %d %d %d",eulers[0],eulers[1],eulers[2]);
		}
	}
	EulerTime euler = new EulerTime();

	static class RangeTime {
		float range;
		long rangeTime = 0L;
	}
	RangeTime ranges = new RangeTime();

	static class MagTime {
		geometry_msgs.Vector3 mag3;
		long magTime = 0L;
		public String toString() {
			return String.format("Mag field: %d %d %d",mag3.getX(),mag3.getY(),mag3.getZ());
		}
	}
	MagTime mags = new MagTime();

	static class AngularTime {
		geometry_msgs.Vector3 angular; 
		geometry_msgs.Vector3 linear; 
		geometry_msgs.Quaternion orientation;
		long angTime = 0L;
		public String toString() {
			return String.format("Nav:Orientation X:%d Y:%d Z:%d W:%d, Linear: %d %d %d, Angular: %d %d %d",orientation.getX(),orientation.getY(),orientation.getZ(),orientation.getW(),
					linear.getX(), linear.getY(), linear.getZ(), angular.getX(), angular.getY(), angular.getZ());
		}
	}
	AngularTime angs = new AngularTime();

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
	public static boolean isTemperature = false;
	public static boolean isOverTemp = false;
	public static boolean isMoving = false;
	public static boolean isNav = false;
	public static boolean isRangeUpperFront = false;
	public static boolean isRangeLowerFront = false;
	public static boolean isVision = false; // machine vision recognition event
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
	// slope speed change over time analysis
	private CircularBlockingDeque<Float> speedQueueL = new CircularBlockingDeque<Float>(5);
	private CircularBlockingDeque<Float> speedQueueR = new CircularBlockingDeque<Float>(5);
	private float maxSpeedSlope = 100;

	static RobotInterface robot;
	public String RPT_SERVICE = "robo_status";
	private CircularBlockingDeque<diagnostic_msgs.DiagnosticStatus> statusQueue = new CircularBlockingDeque<diagnostic_msgs.DiagnosticStatus>(1024);
	MarlinspikeManager marlinspikeManager;
	Collection<DeviceEntry> deviceEntries;
	HashMap<Integer, Boolean> publishedLUNRestValue = new HashMap<Integer, Boolean>();
	boolean[] isActive;

	public MotionController(String[] args) {
		CommandLineLoader cl = new CommandLineLoader(Arrays.asList(args));
		NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
		nodeMainExecutor.execute(this, cl.build());
		if( DEBUG ) {
			System.out.println("Bringing up MotionControl with args:"+args[0]+" "+args[1]+" "+args[2]);
		}
	}
	/**
	 * The constructor will load but not activate the nodes listed in configuration when {@link MarlinspikeManager}.configureMarlinspike
	 * is called with first parameter 'override' set to true, and second parameter 'activate' set to false to indicate we
	 * want to get configuration for all available control nodes, but not activate them from this module since this module
	 * issues directives to the attached controllers rather than control them directly.<p/>
	 * We then create collection of {@link NodeDeviceDemuxer} by calling getNodeDeviceDemuxerByType in the MarlinspikeManager.<p/>
	 * Finally we create the isActive boolean array to hold/control status of devices.
	 * @throws IOException 
	 */
	public MotionController() throws IOException {
		robot = new Robot();
		marlinspikeManager = new MarlinspikeManager(robot);
		marlinspikeManager.configureMarlinspike(true, false);
		deviceEntries = marlinspikeManager.getDevices();
		isActive = new boolean[deviceEntries.size()];
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
		if(remaps.containsKey("__debug"))
			DEBUG = true;
		//if( remaps.containsKey("__serveNode") ) {
		//	serveNode = remaps.get("__serveNode");
		//} else {
		//	throw new RuntimeException("Must specify __serveNode:=<node> to configure topics to publish to node specificed in configuration.");
		//}
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
		// Check startup for indoor speed scale setting ((stick -1 to 1) * SPEEDSCALE), then check cmdl for override of speed scale from startup and defaults
		// MAX is always 1000
		if(robot.isIndoor())
			SPEEDSCALE = 100.0f;
		else
			SPEEDSCALE = 1000.0f;
		if( remaps.containsKey("__speedscale") )
			SPEEDSCALE = Float.parseFloat(remaps.get("__speedscale"));

		final Publisher<diagnostic_msgs.DiagnosticStatus> statpub =
				connectedNode.newPublisher("robocore/status", diagnostic_msgs.DiagnosticStatus._TYPE);

		final HashMap<String, Publisher<std_msgs.Int32MultiArray>> pubschannel = new HashMap<String, Publisher<std_msgs.Int32MultiArray>>();

		for(DeviceEntry ndd : deviceEntries) {
			//if(ndd.getNodeName().equals(serveNode)) {
			Publisher<std_msgs.Int32MultiArray> pub =(connectedNode.newPublisher(ndd.getName(), std_msgs.Int32MultiArray._TYPE));
			pub.addListener(new PublisherListener<std_msgs.Int32MultiArray>() {
				@Override
				public void onMasterRegistrationFailure(Publisher<Int32MultiArray> pub) {
					throw new RuntimeException("Failed to register with master "+pub);					
				}
				@Override
				public void onMasterRegistrationSuccess(Publisher<Int32MultiArray> pub) {
					if(DEBUG) {
						System.out.printf("Successful Master Registration for %s%n", pub);
					}
					pubschannel.put(ndd.getName(), pub);
					if(DEBUG)
						System.out.println("Bringing up publisher "+ndd.getName());
				}
				@Override
				public void onMasterUnregistrationFailure(Publisher<Int32MultiArray> pub) {
					if(DEBUG) {
						System.out.printf("<<FAILED TO UNREGISTER WITH MASTER for %s%n", pub);
					}
				}
				@Override
				public void onMasterUnregistrationSuccess(Publisher<Int32MultiArray> pub) {
					if(DEBUG) {
						System.out.printf("Successful Master Unregistration for %s%n", pub);
					}
					pubschannel.forEach((key,value) -> { if(value == pub) pubschannel.remove(key);});
				}
				@Override
				public void onNewSubscriber(Publisher<Int32MultiArray> pub, SubscriberIdentifier sub) {
					if(DEBUG) {
						System.out.printf("New subscriber for %s Header: %s%n", pub, sub.toConnectionHeader().getFields());
					}				
				}
				@Override
				public void onShutdown(Publisher<Int32MultiArray> pub) {
					if(DEBUG) {
						System.out.printf("Shutdown initiated for %s%n", pub);
					}
					pubschannel.forEach((key,value) -> { if(value == pub) pubschannel.remove(key);});
				}	
			});
			publishedLUNRestValue.put(ndd.getLUN(),false);
		}
		//}	

		final Publisher<geometry_msgs.Twist> twistpub = 
				connectedNode.newPublisher("cmd_vel", geometry_msgs.Twist._TYPE);

		final geometry_msgs.Twist twistmsg = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Twist._TYPE);

		Subscriber<sensor_msgs.Joy> substick = connectedNode.newSubscriber("/sensor_msgs/Joy", sensor_msgs.Joy._TYPE);
		Subscriber<sensor_msgs.Imu> subsimu = connectedNode.newSubscriber("/sensor_msgs/Imu", sensor_msgs.Imu._TYPE);
		final Subscriber<std_msgs.String> subsrange = connectedNode.newSubscriber("/sensor_msgs/range",std_msgs.String._TYPE);
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
			System.out.println("Robot:"+robot);
		ArrayList<String> roboProps = new ArrayList<String>();
		roboProps.add(robot.toString());

		substick.addSubscriberListener(new SubscriberListener<sensor_msgs.Joy>() {
			@Override
			public void onMasterRegistrationFailure(Subscriber<Joy> subs) {
				if(DEBUG)
					System.out.printf("%s Subscsriber %s failed to register with master!%n", this.getClass().getName(), subs);				
			}
			@Override
			public void onMasterUnregistrationSuccess(Subscriber<Joy> subs) {
				if(DEBUG)
					System.out.printf("%s Subscsriber %s unregistered with master!%n", this.getClass().getName(), subs);			
			}
			@Override
			public void onMasterRegistrationSuccess(Subscriber<Joy> subs) {

			}
			@Override
			public void onMasterUnregistrationFailure(Subscriber<Joy> subs) {
				if(DEBUG)
					System.out.printf("%s Subscsriber %s failed to unregister with master!%n", this.getClass().getName(), subs);					
			}
			@Override
			public void onNewPublisher(Subscriber<Joy> subs, PublisherIdentifier pubs) {
				if(DEBUG)
					System.out.printf("%s Subscsriber %s registered with publisher %s!%n", this.getClass().getName(), subs, pubs);
				substick.addMessageListener(new MessageListener<sensor_msgs.Joy>() {
					@Override
					public void onNewMessage(sensor_msgs.Joy message) {
						processJoystickMessages(connectedNode, pubschannel, message, twistpub, twistmsg);
					} // onMessage from Joystick controller, with all the axes[] and buttons[]	
				});
			}
			@Override
			public void onShutdown(Subscriber<Joy> subs) {
				if(DEBUG)
					System.out.printf("%s Subscsriber %s shutdown!%n", this.getClass().getName(), subs);
			}

		});

		subsimu.addMessageListener(new MessageListener<sensor_msgs.Imu>() {
			@Override
			public void onNewMessage(sensor_msgs.Imu message) {
				try {
					synchronized(angs) {
						if(angs.angular == null || message.getAngularVelocity().getX() != angs.angular.getX() ||
								message.getAngularVelocity().getY() != angs.angular.getY() ||
								message.getAngularVelocity().getZ() != angs.angular.getZ() ||
								message.getLinearAcceleration().getX() != angs.linear.getX() ||
								message.getLinearAcceleration().getY() != angs.linear.getY() ||
								message.getLinearAcceleration().getZ() != angs.linear.getZ()) {
							angs.angular = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
							angs.linear = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
							angs.orientation =  connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE);
							angs.orientation.setX(message.getOrientation().getX());
							angs.orientation.setY(message.getOrientation().getY());
							angs.orientation.setZ(message.getOrientation().getZ());
							angs.orientation.setW(message.getOrientation().getW());
							euler.eulers = message.getOrientationCovariance();
							euler.eulerTime = System.currentTimeMillis();
							angs.angTime = System.currentTimeMillis();

							if(IMUDEBUG) {
								System.out.println("Nav:"+euler.toString()+" "+angs.toString());
							}
							isNav = true;
							if( SHOCK_THRESHOLD[0] != -1 ) {
								if( Math.abs(angs.linear.getX()-SHOCK_BASELINE[0]) > SHOCK_THRESHOLD[0] ) {
									isOverShock = true;
								}
								if( Math.abs(angs.linear.getY()-SHOCK_BASELINE[1]) > SHOCK_THRESHOLD[1] ) {
									isOverShock = true;
								}
								if( Math.abs(angs.linear.getZ()-SHOCK_BASELINE[2]) > SHOCK_THRESHOLD[2] ) {
									isOverShock = true;
								}
							}
						} else
							isNav = false;
					}
				} catch (Throwable e) {
					isNav = false;
					//System.out.println("Nav subs exception:"+e.getMessage());
					ArrayList<String> st = new ArrayList<String>();
					st.addAll(Arrays.stream(e.getStackTrace()).map(m->m.toString()).collect(Collectors.toList()));
					new PublishDiagnosticResponse(connectedNode, statpub, statusQueue, "IMU",
							diagnostic_msgs.DiagnosticStatus.ERROR, st);
				}
			}
		},5); // buffer 5 messages

		subsrange.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String message) {
				if( DEBUG )
					System.out.println("New range msg:"+message);
				try {
					if( Float.parseFloat(message.getData()) != rangetop) {
						rangetop = Float.parseFloat(message.getData());
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
		});
		/*
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
				if( IMUDEBUG )
					System.out.println("New mag msg:"+message);
				synchronized(mags) {
					if(mags.mag3 == null)
						mags.mag3 = message.getMagneticField();
					if( mags.mag3.getX() != message.getMagneticField().getX() || 
							mags.mag3.getY() != message.getMagneticField().getY() || 
							mags.mag3.getZ() != message.getMagneticField().getZ()) {
						mags.mag3.setX(message.getMagneticField().getX()); 
						mags.mag3.setY(message.getMagneticField().getY());
						mags.mag3.setZ(message.getMagneticField().getZ());
						mags.magTime = System.currentTimeMillis();
						isMag = true;
						if(IMUDEBUG)
							System.out.println(mags);
						if( MAG_THRESHOLD[0] != -1 ) {
							if( mags.mag3.getX() > MAG_THRESHOLD[0] && mags.mag3.getY() > MAG_THRESHOLD[1] && mags.mag3.getZ() > MAG_THRESHOLD[2] ) {
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
				if( IMUDEBUG )
					System.out.println("New temp msg:"+message);
				if(temperature != message.getTemperature()) {
					temperature = message.getTemperature();
					if(DEBUG || IMUDEBUG)
						System.out.println(" Temp:"+temperature);
					isTemperature = true;
					if( temperature > robot.getTemperatureThreshold() )
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
				robot.getIMUSetpointInfo().setMinimum(-robot.getIMUSetpointInfo().getMaximum()); //degree minimum integral windup
				robot.getMotionPIDController().clearPID();
				// wheelbase refers to max speed, both wheels averaged
				WHEELBASE = (robot.getLeftSpeedSetpointInfo().getMaximum()+robot.getRightSpeedSetpointInfo().getMaximum())/2.0f;
				// default kp,ki,kd;
				//SetTunings(5.55f, 1.0f, 0.5f); // 5.55 scales a max +-180 degree difference to the 0 1000,0 -1000 scale
				//SetOutputLimits(0.0f, SPEEDLIMIT); when pid controller created, max is specified
			}

			@Override
			protected void loop() throws InterruptedException {	
				try {
					awaitStart.await();
				} catch (InterruptedException e) {}
				/*
				hasMoved = motorControlListener.move2DRelative(np.getGyros()[0] , courseOffset, linearMove
						, np.getTimeVal(), np.getAccs(), np.getRanges());
					//System.out.println("Robot should have Moved to "+(robotTheta *= 57.2957795)+" degrees"); // to degrees		
				 */

				// publish messages to status listener if applicable

				if( isOverMag ) {
					isOverMag = false;
					if(DEBUG)
						System.out.println("Mag EXCEEDS threshold..");
					ArrayList<String> magVals = new ArrayList<String>(3);
					synchronized(mags) {
						magVals.add(String.valueOf(mags.mag3.getX()));
						magVals.add(String.valueOf(mags.mag3.getY()));
						magVals.add(String.valueOf(mags.mag3.getZ()));
					}
					new PublishDiagnosticResponse(connectedNode, statpub, statusQueue, "Magnetic anomaly detected", 
							diagnostic_msgs.DiagnosticStatus.WARN, magVals);
				}

				if( isOverShock && !isMoving) {
					if(DEBUG)
						System.out.println("OVERSHOCK!");
					ArrayList<String> shockVals = new ArrayList<String>(2);
					synchronized(angs) {
						shockVals.add(angs.toString());
					}
					new PublishDiagnosticResponse(connectedNode, statpub, statusQueue, "Accelerometer shock warning", 
							diagnostic_msgs.DiagnosticStatus.WARN, shockVals);
				}

				if( isOverPressure ) { // check for dropping
					//long meas = System.currentTimeMillis()-lastPressureNotification;
					//if( meas > 1000000) {
					//	isPress = true;
					//}
					//return;
					//}
					isOverPressure = false;
					ArrayList<String> pressVals = new ArrayList<String>(1);
					pressVals.add(String.valueOf(pressure));
					new PublishDiagnosticResponse(connectedNode, statpub, statusQueue, "Atmospheric pressure warning", 
							diagnostic_msgs.DiagnosticStatus.WARN, pressVals);
				}

				if(isOverTemp) {
					if( DEBUG )
						System.out.println("DANGER Temp:"+temperature);
					isOverTemp = false;
					ArrayList<String> tempVals = new ArrayList<String>(1);
					tempVals.add(String.valueOf(temperature));
					new PublishDiagnosticResponse(connectedNode, statpub, statusQueue, "Temperature warning", 
							diagnostic_msgs.DiagnosticStatus.WARN, tempVals);
				}

				while(!statusQueue.isEmpty()) {
					statpub.publish(statusQueue.takeFirst());
					++sequenceNumber;
					if( DEBUG )
						System.out.println("Sequence:"+sequenceNumber);
					Thread.sleep(1);
				}

			}
		}); // cancellable loop

	} // onStart

	/**
	 * Move the buffered values into the publishing message to send absolute vals to motor and peripheral control.
	 * We will be sending [<number> <> <>]
	 * We use a Int32MultiArray type of 1 dimension, row of each value with other dimensions 0.
	 * the first vale will be the ordinal of the named LUN configuration entry
	 * @param connectedNode Our node transceiver info
	 * @param valBuf The linear array of values to send, which we will be transposing to our MultiAray
	 * @return The multi array we create
	 */
	private Int32MultiArray setupPub(ConnectedNode connectedNode, ArrayList<Integer> vals) {
		return RosArrayUtilities.setupInt32Array(connectedNode, vals);
	}
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
	 * Increase power of left wheel based on PID results. If we max out we are going to apply the leftover
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
			System.out.printf(" LEFTANGLE=%f|chord=%f|hypot=%f|radius/hypot=%f|Hold=%b\n",radius,chord,hypot,((radius+(robot.getDiffDrive().getLeftWheel().getSpeedsetPointInfo().getWheelTrack()/2))/hypot),holdBearing);
		speedR *= ((radius+(WHEELBASE/2))/hypot);
	}

	/**
	 * Process joystick messages. <p/>
	 * The buttons are basically hardwired to predefined functions, and LUN[0] and LUN[1] are typically assigned device names LeftWheel
	 * and RightWheel and the 2 axes of stick specified in the config file are blended into a steering speed and angle based on the 2 axes,
	 * which is published to each wheel channel as a speed in the range -1000 to +1000.<p/>
	 * 
	 * The other device name LUNs can be configured as desired with the values interpolated into a -1000 to +1000 range based on the type
	 * of axis defined and the values(s) published to the DeviceName LUN channel as a Ros int32 multi array.<p/>
	 *
	 * Joystick data will have array of axis and buttons, typically, axis[0] and axis[2] are left stick x,y axis[1] and axis[3] are right.
	 * The code calculates the theoretical speed for each wheel in the 0-1000 scale or SPEEDSCALE based on target point vs IMU yaw angle.
	 * If the joystick chimes in the target point becomes the current course minus relative stick position,
	 * and the speed is modified by the Y value of the stick.
	 * Button presses cause rotation in place or emergency stop or cause the robot to hold to current course, using the IMU to 
	 * correct for deviation an wheel slippage.<p/>
	 * 
	 * To turn, we are going to calculate the arc lengths of the inner wheel and outer wheel based on speed we are presenting by stick y
	 * (speed) forming the radius of the arc and the offset of stick angle x,y degrees from 0, added to current heading, forming theta.
	 * Theta may also be formed by button press, and the difference in current heading and ongoing IMU headings for bearing on a forward course.
	 * We are then going to assume that the distance each wheel has to travel represents a ratio that is also the ratio
	 * of speeds of each wheel, time and speed being distance, and the fact that both wheels, being attached to the robot,
	 * have to arrive at essentially the same time after covering the desired distance based on desired speed.<p/>
	 * 
	 * The net effect that as speed increases the turning radius also increases, as the radius is formed by speed (Y of stick) scaled by 1000
	 * in the case of the inner wheel, and inner wheel plus 'effective robot width' or WHEELBASE as the outer wheel arc radius to traverse.
	 * So we have the theta formed by stick and IMU, and 2 radii formed by stick y and WHEELBASE, and we generate 2 arc lengths that are taken
	 * as a ratio that is multiplied by the proper wheel depending on direction, to reduce power in one wheel, to affect turn.
	 * The ratio of arc lengths depending on speed and turn angle becomes the ratio of speeds at each wheel.<p/>
	 * 
	 * The above method is used for interactive control via joystick and for large granularity correction in autonomous mode.
	 * The same technique is used in autonomous mode for finer correction by substituting the base of a right triangle as the speed 
	 * for the inner arc, and the hypotenuse computed by the base and chord formed from course deviation, and half wheelbase, for the outer arc.
	 * The triangle solution uses radius in the forward direction, rather than at right angles with WHEELBASE as the arcs do, to bring
	 * us into refined tangents to the crosstrack. At final correction, a PID algorithm is used to maintain fine control.
	 * axis[6] is POV, it has quantized values to represent its states.<p/>
	 * 
	 * @param message --float[] axis:--			<br/>
	 * [0] - left horiz (X)						<br/>
	 * [1] - left vert (Y)						<br/>
	 * [2] - right horiz (X)						<br/>
	 * [3] - right vert (Y) 						<br/>
	 * [4] - left trigger						<br/>
	 * [5] - right trigger						<br/>
	 * [6] - POV									<br/>
	 * --int[] buttons--							<br/>
	 * [0] - select								<br/>
	 * [1] - start								<br/>
	 * [2] - left stick press					<br/>
	 * [3] - right stick press					<br/>
	 * [4] - triangle -- HOLD CURRENT BEARING	<br/>
	 * [5] - circle -- RIGHT PIVOT				<br/>
	 * [6] - X button -- EMERGENCY STOP			<br/>
	 * [7] - square -- LEFT PIVOT				<br/>
	 * [8] - left bumper							<br/>
	 * [9] - right bumper						<br/>
	 * -- joystick only --						<br/>
	 * [10] - back								<br/>
	 * [11] - extra								<br/>
	 * [12] - mode								<br/>
	 * [13] - side								<br/>
	 * @param connectedNode The Ros node we are using here
	 * @param pubschannel The map of publisher channels by LUN device name from collection of {@link NodeDeviceDemuxer}
	 * @param message The joystick message with all buttons and axes
	 * @param twistpub The twist publisher channel for broadcast messages to all devices for emergency stop, etc.
	 * @param twistmsg The twist message template to populate with data from this method
	 */
	private void processJoystickMessages(ConnectedNode connectedNode, HashMap<String, Publisher<Int32MultiArray>> pubschannel, Joy message, 
			Publisher<geometry_msgs.Twist> twistpub, geometry_msgs.Twist twistmsg) {
		if(DEBUG) {
			System.out.printf("%s Axes:%s Buttons:%s%n", this.getClass().getName(),Arrays.toString(message.getAxes()),Arrays.toString(message.getButtons()));
		}
		float[] axes = message.getAxes();
		int[] buttons = message.getButtons();
		final geometry_msgs.Vector3 angular = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
		final geometry_msgs.Vector3 linear = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
		//final geometry_msgs.Quaternion orientation =  connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE);
		// check for emergency stop, on X or A or green or lower button
		if( buttons[6] != 0 ) {
			if(DEBUG)
				System.out.println("**EMERGENCY STOP **");
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
		// Process the affectors and peripherals before the motion controls, this is mainly due to some of the logic
		// In motion control returning from this method rather than try to implement more complex decision making.
		// See if the triggers were activated. Axes[4] and axes[5] are the left and right triggers.
		// Check them and see if either one was depressed. If so, scale them to the -1000 to 1000
		// SPEEDSCALE constant (or whatever the SPEEDSCALE constant is, we presume its set at 1000)
		// for the majority of downstream processing. In the case of PWM, we are going to scale this
		// from -1000,1000 to 0,2000 since controls such as LED dont have a negativer or 'reverse' value.
		// Actually, could be potentially destructive to reverse polarity as a motor does, so we are
		// sure to scale it to the positive range downstream. We are going to publish the scaled
		// values to absolute/cmd_periph1 and let the downstream processing handle further scaling
		// if necessary. If we reach an off state of -1, we want to send it anyway to turn off the LED, 
		// hence the boolean checks.
		//
		// Start the motion processing for the differential drive using joystick axes[0] and [2] left stick
		// If the button square or circle is depressed, rotate in place at stick position Y speed
		if( buttons[7] != 0 ) { // left pivot
			speedR = -axes[robot.getDiffDrive().getControllerAxisY()] * SPEEDSCALE;
			speedL = -speedR;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
			// set it up to send
			publishPropulsion(connectedNode, pubschannel, (int)speedL, (int)speedR);
			if(DEBUG)
				System.out.printf("Stick Left pivot speedL=%f|speedR=%f\n",speedL,speedR);
			return;
		} else {
			if( buttons[5] != 0 ) { // right pivot
				speedL = -axes[robot.getDiffDrive().getControllerAxisY()] * SPEEDSCALE;
				speedR = -speedL;
				// set it up to send
				publishPropulsion(connectedNode, pubschannel, (int)speedL, (int)speedR);
				if(DEBUG)
					System.out.printf("Stick Right pivot speedL=%f|speedR=%f\n",speedL,speedR);
				return;
			}
		}
		// get radian measure of left stick x,y
		float radians = (float) Math.atan2(axes[robot.getDiffDrive().getControllerAxisY()], axes[robot.getDiffDrive().getControllerAxisX()]);
		float stickDegrees = (float) (radians * (180.0 / Math.PI)); // convert to degrees

		// horizontal axis is 0 to -180/180 degrees, we want 0 at the top
		//         -90
		//          |
		//-180/180 --- 0
		//          |
		//         90
		stickDegrees += 90; // fixes 2 right quadrants and top left quadrant, orients 0 at top, 180 at bottom, 0 to -90 top left.
		if(stickDegrees > 180.0 ) // fixes bottom left quadrant, sets -90 to -180
			stickDegrees -= 360.0;

		// we have absolute stickdegrees, offset the current IMU by that quantity to get new desired course
		float offsetDegrees;
		synchronized(euler) {
			offsetDegrees = (float) (euler.eulers[0] - stickDegrees);
		}
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
				// Setpoint is desired target yaw angle from IMU,vs heading minus the joystick offset from 0
				synchronized(euler) {
					robot.getIMUSetpointInfo().setDesiredTarget((float) euler.eulers[0]);
				}
				robot.getIMUSetpointInfo().setTarget((float)0);
				robot.getMotionPIDController().clearPID();
				wasPid = true; // start with PID control until we get out of tolerance
				if(DEBUG);
				System.out.println("Stick absolute deg set:"+robot.getIMUSetpointInfo());	
			}
		} else {
			holdBearing = false;
			// joystick setting becomes desired target yaw angle heading and IMU is disregarded
			robot.getIMUSetpointInfo().setDesiredTarget(stickDegrees);
			robot.getIMUSetpointInfo().setTarget((float)0); 
		}

		// eulers[0] is Input: target yaw angle from IMU. If IMU is delivering a 0, then our delta below will just be desiredTarget.
		synchronized(euler) {
			robot.getIMUSetpointInfo().setTarget((float) euler.eulers[0]); 
		}
		//
		// perform the PID processing
		// IMUSetpointInfo looks like this:
		// public void setTarget(float t) { yawAngle = t;	}
		// public float getTarget() { return yawAngle; }
		// public void setDesiredTarget(float t) { desiredYawAngle = t; }
		// public float getDesiredTarget() { return desiredYawAngle; }
		// public float delta() { return desiredYawAngle - yawAngle; }
		//
		// MotionPidController.Compute(SetPointInfo) does this with SetPointInfo before it sets PID terms:
		// SetPointInfo.setPrevErr(error);
		// error = SetPointInfo.delta();
		// error =  error % 360; // angle or 360 - angle
		// if (error <= -180.0)
		//     error += 360.0;
		// if (error >= 180)  
		//     error -= 360;  
		//
		((MotionPIDController) robot.getMotionPIDController()).Compute(robot.getIMUSetpointInfo());

		// Speed may be plus or minus, this determines a left or right turn as the quantity is added to desired speed
		// of wheels left and right. The speed for each wheel is modified by desired turn radius, which is based on speed,
		// or on the computed derivation from desired course if a straight line course is set.
		// axes[2] is y, value is -1 to 0 to 1, and for some reason forward is negative on the stick
		// scale it from -1 to 0 to 1 to -1000 to 0 to 1000, or the value of SPEEDSCALE which is our speed control range 
		speedR = -axes[robot.getDiffDrive().getControllerAxisY()] * SPEEDSCALE;
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
		float radius = Math.abs(axes[robot.getDiffDrive().getControllerAxisY()]) * SPEEDSCALE;
		float arcin = 0, arcout = 0;
		//
		if( holdBearing ) {
			if(DEBUG || DEBUGBEARING) {
				System.out.printf("HOLDING BEARING %s\n",robot.getMotionPIDController().toString());
				synchronized(euler) {
					System.out.printf("Stick deg=%f|Offset deg=%f|IMU=%f|arcin=%f|arcout=%f|speedL=%f|speedR=%f\n",stickDegrees,offsetDegrees,euler.eulers[0],arcin,arcout,speedL,speedR);
				}
			}
			followIMUSetpoint(radius, arcin, arcout);
		} else {
			// manual steering mode, use tight radii and a human integrator
			// Compute arc length on inner and outer wheel
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
			speedL = slope(speedL, speedQueueL);
			speedR = slope(speedR, speedQueueR);
			if(DEBUG || DEBUGBEARING) {
				System.out.printf("MANUAL BEARING %s\n",robot.getMotionPIDController().toString());
				synchronized(euler) {
					System.out.printf("Stick deg=%f|Offset deg=%f|IMU=%f|arcin=%f|arcout=%f|speedL=%f|speedR=%f\n",stickDegrees,offsetDegrees,euler.eulers[0],arcin,arcout,speedL,speedR);
				}
			}
		}

		publishPropulsion(connectedNode, pubschannel, (int)speedL, (int)speedR);

		publishPeripheral(connectedNode, pubschannel, axes);

	}
	/**
	 * If slope is negative and coordinate is neg acceleration in reverse
	 * If slope is pos and coord is negative reverse deceleration
	 * 
	 * @param speed
	 * @param speedQueue
	 * @return
	 */
	private float slope(float speed, CircularBlockingDeque<Float> speedQueue) {
		speedQueue.addLast(speed);
		int n = speedQueue.length();
		double m, c, sum_x = 0, sum_y = 0, sum_xy = 0, sum_x2 = 0;
		for (int i = 0; i < n; i++) {
			sum_x += i;//x[i];
			sum_y += speedQueue.get(i);//[i];
			sum_xy += i * speedQueue.get(i);//[i] * y[i];
			sum_x2 += Math.pow(i, 2);//pow(x[i], 2);
		}
		m = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - Math.pow(sum_x, 2));
		//c = (sum_y - m * sum_x) / n;
		//System.out.println("m = " + m);
		//System.out.println("c = " + c);
		if(m == 0)
			return speedQueue.get(n-1);
		if(m < 0) {
			if(m < -maxSpeedSlope) {
				if(n == 1) {
					speedQueue.set(0,speedQueue.get(1)/2); // divide it in half, we only have 1 point
				} else {
					// slope neg and coord neg, reverse accel
					if(speedQueue.get(n-1) < 0) {
						// diff of last point and next to last
						float diff = speedQueue.get(n-1)-speedQueue.get(n-2);
						// split the diff
						speedQueue.set(n-1, speedQueue.get(n-2)+(diff/2.0f));
					} else {
						// slope neg and coord pos, forward decel
						// diff of next to last and last
						float diff = speedQueue.get(n-2)-speedQueue.get(n-1);
						// split the diff
						speedQueue.set(n-1, speedQueue.get(n-2)-(diff/2.0f));
					}
				}
			}
		} else {
			// slope pos
			if(m > maxSpeedSlope) {
				if(n == 1) {
					speedQueue.set(0,speedQueue.get(1)/2); // divide it in half, we only have 1 point
				} else {
					// slope pos and coord neg, reverse decel
					if(speedQueue.get(n-1) < 0) {
						// diff of next to last and last
						float diff = speedQueue.get(n-2)-speedQueue.get(n-1);
						// split the diff
						speedQueue.set(n-1, speedQueue.get(n-2)-(diff/2.0f));
					} else {
						// slope pos and coord pos, forward accel
						// diff of last and next to last
						float diff = speedQueue.get(n-1)-speedQueue.get(n-2);
						// split the diff
						speedQueue.set(n-1, speedQueue.get(n-2)+(diff/2.0f));
					}
				}
			}
		}
		return speedQueue.get(n-1);
	}
	/**
	 * 
	 * Process other axis.<p/>
	 * Get the list of axis and or buttons according to LUN and publish to each.
	 * We are going to translate the floating values to integer by scaling them * 1000 as they are typically in the 0-1 range.
	 * We scale the POV trigger to +1000, -1000 based on AxisUp and AxisDown if we want it to act as a simple up/down control.<br/>
	 * The DigitalHat POV is 8 positions, left being 1.0, subtracting .125 counterclockwise, so straight up is .25, straight down is .75<p/>
	 * Configuration:			<br/>
	 * AXIS[0].AxisType:Stick	<br/>
	 * AXIS[0].AxisX:0			<br/>
	 * AXIS[0].AxisY:2			<br/>
	 * ---						<br/>
	 * AXIS[3].AxisType:Trigger<br/>
	 * AXIS[3].Axis:4			<br/>
	 * ---						<br/>
	 * AXIS[4].AxisType:POV	<br/>
	 * AXIS[4].Axis:6			<br/>
	 * -- optional:				<br/>
	 * AXIS[4].AxisUp:0.25		<br/>
	 * AXIS[4].AxisDown:0.75	<br/>
	 *<br/>
	 * Assume we handle propulsion in other method, so ignore the LUN device names that end in 'Wheel' and process all others, extracting the axis
	 * definitions, retrieving the values from the axes array, and publishing them to the LUN.name channels by:<br/>
	 * <code>getPublisher(pubschannel, LUN.name).publish(setupPub(connectedNode, ArrayList<Integer>(vals)));</code>
	 * @param connectedNode Our Ros node here
	 * @param pubschannel Collection of publisher channels that represent axes
	 * @param axes the values of the stick axes
	 */
	private void publishPeripheral(ConnectedNode connectedNode, HashMap<String, Publisher<Int32MultiArray>> pubschannel, float[] axes) {
		deviceEntries.forEach( lun -> {
			if(!(lun.getName().endsWith("Wheel"))) {
				int luni = lun.getLUN();
				String axisType = (String) robot.getAXIS()[luni].get("AxisType");
				if(axisType == null) {
					if(DEBUG) {
						System.out.printf("%s NO axis type attribute device %s%n", this.getClass().getName(), lun);
					}
					return; // continue with next iteration
				}
				if(DEBUG) {
					System.out.printf("%s axis type %s attribute device %s%n", this.getClass().getName(), axisType, lun);
				}
				switch(axisType) {
				case "Stick":
					if(robot.getAXIS()[luni].get("AxisX") != null) {
						String sx = (String) robot.getAXIS()[luni].get("AxisX");
						String sy = (String) robot.getAXIS()[luni].get("AxisY");
						float x = -axes[Integer.parseInt(sx)] * 1000;
						float y = -axes[Integer.parseInt(sy)] * 1000;
						if( y != 0 || x != 0) {
							publishAxis(connectedNode, pubschannel, lun.getName(), (int)y, (int)x);
							publishedLUNRestValue.replace(luni, false);
						} else {
							// rest value here
							if(!publishedLUNRestValue.get(luni)) {
								publishAxis(connectedNode, pubschannel, lun.getName(), (int)y, (int)x); // not published, so hit once
								publishedLUNRestValue.replace(luni, true); // and set it as published at rest
							}
						}
					} else {
						String sy = (String) robot.getAXIS()[luni].get("AxisY");
						float y = -axes[Integer.parseInt(sy)] * 1000;
						if( y != 0) {
							publishAxis(connectedNode, pubschannel, lun.getName(), (int)y);
							publishedLUNRestValue.replace(luni, false); // rest value not yet sent
						} else {
							// rest value here
							if(!publishedLUNRestValue.get(luni)) {
								publishAxis(connectedNode, pubschannel, lun.getName(), (int)y); // rest not yet published, so do it
								publishedLUNRestValue.replace(luni, true); // rest value sent
							}
						}
					}
					break;
				case "Trigger":
					String ax = (String) robot.getAXIS()[luni].get("Axis");
					float a = axes[Integer.parseInt(ax)] * 1000;
					if(a != -1000) {
						publishAxis(connectedNode, pubschannel, lun.getName(), (int)a);
						publishedLUNRestValue.replace(luni, false);
					} else {
						// rest value here
						if(!publishedLUNRestValue.get(luni)) {
							publishAxis(connectedNode, pubschannel, lun.getName(), (int)a); // not published, so hit it
							publishedLUNRestValue.replace(luni, true); // and set it as published at rest
						}
					}
					break;
				case "POV":
					ax = (String) robot.getAXIS()[luni].get("Axis");
					a = axes[Integer.parseInt(ax)] * 1000;
					String saxUp = (String) robot.getAXIS()[luni].get(("AxisUp"));
					if(saxUp == null) {
						publishAxis(connectedNode, pubschannel, lun.getName(), (int)a);
					} else {
						String saxDown = (String) robot.getAXIS()[luni].get(("AxisDown"));
						float axUp = Float.parseFloat(saxUp) * 1000;
						float axDown = Float.parseFloat(saxDown) * 1000;
						if(a == axUp) {
							publishAxis(connectedNode, pubschannel, lun.getName(), 1000);
						} else {
							if(a == axDown) {
								publishAxis(connectedNode, pubschannel, lun.getName(), -1000);
							} else {
								if(a == 0) {
									publishAxis(connectedNode, pubschannel, lun.getName(), 0);
								}
							}
						}
					}
					break;
				default:
					throw new RuntimeException("Unknown Axis type in configuration:"+axisType+" for LUN "+lun.getName());
				}
			}
		});	
	}

	/**
	 * Send the motor speed values down the wheel channels
	 * @param connectedNode
	 * @param pubschannel
	 */
	private void publishPropulsion(ConnectedNode connectedNode, HashMap<String, Publisher<Int32MultiArray>> pubschannel, int leftSpeed, int rightSpeed) {
		ArrayList<Integer> speedVals = new ArrayList<Integer>();
		speedVals.add(leftSpeed);
		pubschannel.get("LeftWheel").publish(setupPub(connectedNode, speedVals));
		try {
			Thread.sleep(1);
		} catch (InterruptedException e) {}		
		speedVals.clear();
		speedVals.add(rightSpeed);
		pubschannel.get("RightWheel").publish(setupPub(connectedNode, speedVals));
		try {
			Thread.sleep(1);
		} catch (InterruptedException e) {}
	}
	/**
	 * Send the motor speed values down the wheel channels
	 * @param connectedNode
	 * @param pubschannel
	 */
	private void publishAxis(ConnectedNode connectedNode, HashMap<String, Publisher<Int32MultiArray>> pubschannel, String channel, int... vals) {
		ArrayList<Integer> axisVals = new ArrayList<Integer>();
		for(int v : vals)
			axisVals.add(v);
		if(DEBUG)
			System.out.printf("%s Publish axis channel %s sending values %s%n" , this.getClass().getName(), channel, Arrays.toString(vals));
		pubschannel.get(channel).publish(setupPub(connectedNode, axisVals));
		try {
			Thread.sleep(1);
		} catch (InterruptedException e) {}
	}
	/**
	 * 		
	 * If holding an inertially guided course, check for the tolerance of error proportion
	 * from setpoint to current heading, if within a lower tolerance, use the pid values of the
	 * currently computed course deviation to
	 * alter the motor speed to reduce cross track error and become critically damped.
	 * If we are at the higher tolerance, compute a solution based on triangles that brings us on a tangent course
	 * to the crosstrack. The base of a right triangle is the speed plus half wheelbase, the hypotenuse is computed from
	 * the chord of the magnitude error offset from track and the base (speed). The ratio of base to hypotenuse is used
	 * based on required distance traveled for each wheel and the opposite wheel is slowed by that proportion
	 * just as we do for a solution of arcs.
	 * The chord leg of the triangle is computed based on half wheelbase as radius via 2*R*sin(theta/2) with theta
	 * being crosstrack deviation.
	 * @param radius
	 * @param arcin
	 * @param arcout
	 */
	private void followIMUSetpoint(float radius, float arcin, float arcout) {
		if(DEBUG || DEBUGBEARING)
			System.out.printf("Inertial Setpoint=%f | Hold=%b ", robot.getIMUSetpointInfo().getDesiredTarget(), holdBearing);
		// In auto
		// Point at which geometric solution begins/disengages is IMU maximumCourseDeviation from configuration
		float PID_THRESHOLD  = robot.getIMUSetpointInfo().getMaximum()/2; // point at which PID engages/disengages
		if( Math.abs(robot.getMotionPIDController().getError()) <= robot.getIMUSetpointInfo().getMaximum() ) {
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
			if(DEBUG || DEBUGBEARING)
				synchronized(euler) {
					System.out.printf("<="+robot.getIMUSetpointInfo().getMaximum()+" degrees Speed=%f|IMU=%f|speedL=%f|speedR=%f|Hold=%b\n",radius,euler.eulers[0],speedL,speedR,holdBearing);
				}
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
			if(DEBUG || DEBUGBEARING)
				synchronized(euler) {
					System.out.printf(">"+robot.getIMUSetpointInfo().getMaximum()+" degrees Speed=%f|IMU=%f|arcin=%f|arcout=%f|speedL=%f|speedR=%f|Hold=%b\n",radius,euler.eulers[0],arcin,arcout,speedL,speedR,holdBearing);
				}
		}
	}

	/**
	 * Attempt to travel the target distance in the target time arriving at a final target degrees on the IMU
	 * Here, targetDegrees can be negative
	 * when the IMU and target coincide distance and time produce linear motion
	 * otherwise when turning distance determines the radius of the arc segment described by the difference
	 * of the current IMU and target and time becomes roughly the speed at which to make such traversal
	 * @param yawIMUDegrees Yaw is delivered in degrees
	 * @param yawTargetDegrees target final pose is in degrees for easier human input and represents motion relative to current pose vs absolute position
	 * @param targetDistance target distance to travel in CM, if 0 perform a pivot
	 * @throws IOException 
	 */
	public synchronized void moveRobotRelative(float yawIMUDegrees, float yawTargetDegrees, int targetDistance) throws IOException {
		//while (true) {

		//leftWheel.targetMM = (float)(left_velocity) / clicks_per_mm;
		//rightWheel.targetMM = (float)(right_velocity) / clicks_per_mm;

		//avg_mm = (float) ((leftWheel.targetMM + rightWheel.targetMM) / 2.0);
		//total_mm += avg_mm;
		// wheel_theta is travel of each wheel with desired velocity, if same angle is 0, forward
		//twistInfo.wheelTheta = (leftWheel.targetMM - rightWheel.targetMM) / wheelTrack;
		float wheelTheta = 0.0f;
		float imuTheta = 0.0f;
		//float yawDegrees = 0.0f;
		//float deltaTheta = 0.0f;
		float robotTheta = 0.0f;
		wheelTheta = ((float)yawTargetDegrees * 0.0174532925f); // degr to radians
		/* read the YAW value from the imu struct and convert to radians */
		imuTheta = ((float) (yawIMUDegrees * 0.0174532925f));
		// if final theta negative, turn left
		robotTheta = imuTheta - wheelTheta;
		robotTheta = (float) Math.atan2(Math.sin(robotTheta), Math.cos(robotTheta));
		/* calculate rotation rate-of-change
		deltaTheta = imuTheta - last_theta;
		last_theta = imuTheta;
		robotTheta = imuTheta + wheelTheta;
		// this will generate the value to turn us properly
		if( robotTheta > TWOPI )
			robotTheta -= TWOPI;
		if( robotTheta < 0)
			robotTheta += TWOPI;
		float X =  (float)(targetDistance * Math.sin(robotTheta)); 
		float Y = (float)(targetDistance * Math.cos(robotTheta)); 
		*/
		// so X is the arc radius, normally we will set as a call to
		// setMotorSpeed as 0 to turn in place or a value of arc radius
		// modified by current speed to make a gentle curve.
		// if theta is 0, move linear in x
		// if x and theta not 0 its rotation about a point in space
		//stasis(fabs(wheel_theta),fabs(imu_theta));
		// slot, channel 1, slot, channel 2, theta, yaw radians
		setMotorArcSpeed(robotTheta, targetDistance);
	}

	/**
	 * The function computes motor speeds.
	 * results in leftWheel.targetSpeed and rightWheel.targetSpeed
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
		if(robot.isIndoor()) {
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
	public static void main(String[] args) throws IOException {
		MotionController mc = new MotionController();
		ArrayList<String> pubs = new ArrayList<String>();
		for(DeviceEntry ndd : mc.deviceEntries) {
			System.out.println(ndd.getName());
			if(DEBUG)
				System.out.println("Bringing up publisher "+ndd.getName());
			pubs.add("/"+ndd.getName());
		}
		System.out.printf("Diffydrive; %d %d %n", robot.getDiffDrive().getControllerAxisX(), robot.getDiffDrive().getControllerAxisY());
		System.out.println("----");
		System.out.println(pubs.stream().filter(e -> e.toString().contains("LEDDriver"))
				.findFirst().get());
		System.out.println(pubs.stream().filter(e -> e.toString().contains("BoomActuator"))
				.findFirst().get());
		System.out.println(pubs.stream().filter(e -> e.toString().contains("LeftWheel"))
				.findFirst().get());
		System.out.println(pubs.stream().filter(e -> e.toString().contains("LiftActuator"))
				.findFirst().get());
	}
}
