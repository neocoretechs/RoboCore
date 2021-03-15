package com.neocoretechs.robocore.machine;

import java.io.File;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Map;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;

import org.ros.concurrent.CancellableLoop;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.internal.node.topic.PublisherIdentifier;
import org.ros.internal.node.topic.SubscriberIdentifier;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.service.CountDownServiceServerListener;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.PublisherListener;
import org.ros.node.topic.Subscriber;
import org.ros.node.topic.SubscriberListener;

import com.neocoretechs.robocore.MegaPubs;
import com.neocoretechs.robocore.RosArrayUtilities;
import com.neocoretechs.robocore.PID.IMUSetpointInfo;
import com.neocoretechs.robocore.PID.MotionPIDController;
import com.neocoretechs.robocore.config.Robot;
import com.neocoretechs.robocore.config.RobotInterface;
import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;
import com.neocoretechs.robocore.marlinspike.MarlinspikeManager;
import com.neocoretechs.robocore.marlinspike.NodeDeviceDemuxer;
import com.neocoretechs.robocore.marlinspike.PublishDiagnosticResponse;
import com.neocoretechs.robocore.propulsion.TwistInfo;
import com.neocoretechs.robocore.services.ControllerStatusMessage;
import com.neocoretechs.robocore.services.ControllerStatusMessageRequest;
import com.neocoretechs.robocore.services.ControllerStatusMessageResponse;

import diagnostic_msgs.DiagnosticStatus;
import geometry_msgs.Twist;
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
 * becomes the ratio of speeds.
 * 
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2017,2018,2019,2020
 *
 */
public class PeripheralController extends AbstractNodeMain {
	private static boolean DEBUG = true;
	private String host;
	private InetSocketAddress master;
	private CountDownLatch awaitStart = new CountDownLatch(1);
	
	private String serveNode = null;

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
	public static boolean isTemperature = false;
	public static boolean isOverTemp = false;
	public static boolean isMoving = false;
	public static boolean isNav = false;
	public static boolean isRangeUpperFront = false;
	public static boolean isRangeLowerFront = false;
	public static boolean isVision = false; // machine vision recognition event

	
	boolean hasData = false; // have we received any feedback from callback?
	boolean init = true;

	static long lastTime = System.currentTimeMillis();
	static int SampleTime = 100; //.1 sec
	static float outMin, outMax;
	static boolean inAuto = false;
	static boolean outputNeg = false; // used to prevent integral windup by resetting ITerm when crossing track
	static boolean wasPid = true; // used to determine crossing from geometric to PID to preload integral windup
	
	Object rngMutex1 = new Object();
	Object rngMutex2 = new Object();
	Object navMutex = new Object();
	Object magMutex = new Object();
	
	static RobotInterface robot;
	static {
		try {
			robot = new Robot();
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}
	public String RPT_SERVICE = "robo_status";
	private CircularBlockingDeque<diagnostic_msgs.DiagnosticStatus> statusQueue = new CircularBlockingDeque<diagnostic_msgs.DiagnosticStatus>(1024);
	MarlinspikeManager marlinspikeManager;
	Collection<NodeDeviceDemuxer> listNodeDeviceDemuxer;
	boolean[] isActive;
	/**
	 * We really only use these methods if we want to pull remapped params out of command line or do
	 * some special binding, otherwise the default uses the ROS_HOSTNAME environment or the remapped __ip:= and __master:=
	 * @param host
	 * @param master
	 */
	public PeripheralController(String host, InetSocketAddress master) {
		this.host = host;
		this.master = master;
		NodeConfiguration nc = build();
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, nc);
	    if( DEBUG ) {
	    	System.out.println("Bringing up PeripheralControl with host and master:"+host+" "+master);
	    }   
	}
	
	public PeripheralController(String[] args) {
		CommandLineLoader cl = new CommandLineLoader(Arrays.asList(args));
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, cl.build());
	    if( DEBUG ) {
	    	System.out.println("Bringing up PeripheralControl with args:"+args[0]+" "+args[1]+" "+args[2]);
	    }
	}
	/**
	 * @throws IOException 
	 */
	public PeripheralController() throws IOException {
		marlinspikeManager = new MarlinspikeManager(robot);
		marlinspikeManager.configureMarlinspike(true, false);
		listNodeDeviceDemuxer = marlinspikeManager.getNodeDeviceDemuxerByType(marlinspikeManager.getTypeSlotChannelEnable());
		isActive = new boolean[listNodeDeviceDemuxer.size()];
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
		return GraphName.of("pubsubs_periph");
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
		if( remaps.containsKey("__serveNode") ) {
			serveNode = remaps.get("__serveNode");
		} else {
			throw new RuntimeException("Must specify __serveNode:=<node> to configure topics to publish to node specificed in configuration.");
		}
		if( remaps.containsKey("__kp") )
			robot.getMotionPIDController().setKp(Float.parseFloat(remaps.get("__kp")));
		if( remaps.containsKey("__kd") )
			robot.getMotionPIDController().setKd(Float.parseFloat(remaps.get("__kd")));
		//final Publisher<geometry_msgs.Twist> mopub = connectedNode.newPublisher("cmd_vel", geometry_msgs.Twist._TYPE);
		// statpub has status alerts that may come from sensors.
	
		
		final Publisher<diagnostic_msgs.DiagnosticStatus> statpub =
				connectedNode.newPublisher("robocore/status", diagnostic_msgs.DiagnosticStatus._TYPE);
		
		final Collection<Publisher<std_msgs.Int32MultiArray>> pubschannel = new ArrayList<Publisher<std_msgs.Int32MultiArray>>();
		for(NodeDeviceDemuxer ndd : listNodeDeviceDemuxer) {
			if(ndd.getNodeName().equals(serveNode)) {
				Publisher<std_msgs.Int32MultiArray> pub =(connectedNode.newPublisher("absolute/"+ndd.getDeviceName(), std_msgs.Int32MultiArray._TYPE));
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
						pubschannel.add(pub);
						if(DEBUG)
							System.out.println("Bringing up publisher absolute/"+ndd.getDeviceName());
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
						pubschannel.remove(pub);
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
						pubschannel.remove(pub);
					}	
				});
			}
		}	
		
		final Publisher<geometry_msgs.Twist> twistpub = 
				connectedNode.newPublisher("cmd_vel", geometry_msgs.Twist._TYPE);
		
		final geometry_msgs.Twist twistmsg = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Twist._TYPE);

		Subscriber<sensor_msgs.Joy> subsrange = connectedNode.newSubscriber("/sensor_msgs/Joy", sensor_msgs.Joy._TYPE);
		Subscriber<sensor_msgs.Imu> subsimu = connectedNode.newSubscriber("/sensor_msgs/Imu", sensor_msgs.Imu._TYPE);
		//Subscriber<sensor_msgs.Range> subsrangetop = connectedNode.newSubscriber("UpperFront/sensor_msgs/Range", sensor_msgs.Range._TYPE);
		//Subscriber<sensor_msgs.Range> subsrangebot = connectedNode.newSubscriber("LowerFront/sensor_msgs/Range", sensor_msgs.Range._TYPE);
		Subscriber<sensor_msgs.MagneticField> subsmag = connectedNode.newSubscriber("/sensor_msgs/MagneticField", sensor_msgs.MagneticField._TYPE);
		Subscriber<sensor_msgs.Temperature> substemp = connectedNode.newSubscriber("/sensor_msgs/Temperature", sensor_msgs.Temperature._TYPE);

		if(DEBUG)
			System.out.println("Robot:"+robot);
		ArrayList<String> roboProps = new ArrayList<String>();
		roboProps.add(robot.toString());
		
		final CountDownServiceServerListener<ControllerStatusMessageRequest, ControllerStatusMessageResponse> serviceServerListener =
		        CountDownServiceServerListener.newDefault();
		final ServiceServer<ControllerStatusMessageRequest, ControllerStatusMessageResponse> serviceServer = connectedNode.newServiceServer(RPT_SERVICE , ControllerStatusMessage._TYPE,
		    new ServiceResponseBuilder<ControllerStatusMessageRequest, ControllerStatusMessageResponse>() {
				@Override
				public void build(ControllerStatusMessageRequest request,ControllerStatusMessageResponse response) {	
					//switch(request.getData())
					new PublishDiagnosticResponse(connectedNode, statpub, statusQueue, "Robot Properties", 
							diagnostic_msgs.DiagnosticStatus.OK, roboProps);
				}
			});	
		serviceServer.addListener(serviceServerListener);	      
		try {
			serviceServerListener.awaitMasterRegistrationSuccess(1, TimeUnit.SECONDS);
		} catch (InterruptedException e1) {
			System.out.println("REPORT SERVICE REGISTRATION WAS INTERRUPTED");
			e1.printStackTrace();
		}
		subsrange.addSubscriberListener(new SubscriberListener<sensor_msgs.Joy>() {
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
				if(DEBUG)
					System.out.printf("%s Subscsriber %s registered with master!%n", this.getClass().getName(), subs);	
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
				subsrange.addMessageListener(new MessageListener<sensor_msgs.Joy>() {
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
		
		substemp.addMessageListener(new MessageListener<sensor_msgs.Temperature>() {
			@Override
			public void onNewMessage(sensor_msgs.Temperature message) {
				if(temperature != message.getTemperature()) {
					temperature = message.getTemperature();
					if(DEBUG)
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
			}

			@Override
			protected void loop() throws InterruptedException {	
			    try {
					awaitStart.await();
				} catch (InterruptedException e) {}
				
				while(!statusQueue.isEmpty()) {
					statpub.publish(statusQueue.takeFirst());
					++sequenceNumber;
					if( DEBUG )
						System.out.println("Sequence:"+sequenceNumber);
					Thread.sleep(1);
				}
				
		}
	}); // cancellable loop

	}
	
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
	* Megapubs.typeNames:
	* 0 = LeftWheel
	* 1 = RightWheel
	* 2 = BOOMACTUATOR
	* 3 = LEDDriver
	* 4 = LIFTACTUATOR
	* 5 = ULTRASONIC
	* 6 = PWM
	*
	* Joystick data will have array of axis and buttons, axis[0] and axis[2] are left stick x,y axis[1] and axis[3] are right.
	* The code calculates the theoretical speed for each wheel in the 0-1000 scale or SPEEDSCALE based on target point vs IMU yaw angle.
	* If the joystick chimes in the target point becomes the current course minus relative stick position,
	* and the speed is modified by the Y value of the stick.
	* Button presses cause rotation in place or emergency stop or cause the robot to hold to current course, using the IMU to 
	* correct for deviation an wheel slippage.
	* To turn, we are going to calculate the arc lengths of the inner wheel and outer wheel based on speed we are presenting by stick y
	* (speed) forming the radius of the arc and the offset of stick angle x,y degrees from 0 added to current heading forming theta.
	* Theta may also be formed by button press and the difference in current heading and ongoing IMU headings for bearing on a forward course.
	* We are then going to assume that the distance each wheel has to travel represents a ratio that is also the ratio
	* of speeds of each wheel, time and speed being distance and all, and the fact that both wheels, being attached to the robot,
	* have to arrive at essentially the same time after covering the desired distance based on desired speed.
	* The net effect that as speed increases the turning radius also increases, as the radius is formed by speed (Y of stick) scaled by 1000
	* in the case of the inner wheel, and inner wheel plus 'effective robot width' or WHEELBASE as the outer wheel arc radius to traverse.
	* So we have the theta formed by stick and IMU, and 2 radii formed by stick y and WHEELBASE and we generate 2 arc lengths that are taken
	* as a ratio that is multiplied by the proper wheel depending on direction to reduce power in one wheel to affect turn.
	* The ratio of arc lengths depending on speed and turn angle becomes the ratio of speeds at each wheel.
	* The above method is used for interactive control via joystick and for large granularity correction in autonomous mode.
	* The same technique is used in autonomous mode for finer correction by substituting the base of a right triangle as the speed 
	* for the inner arc and the hypotenuse computed by the base and chord formed from course deviation and half wheelbase for the outer arc.
	* The triangle solution uses radius in the forward direction, rather than at right angles with WHEELBASE as the arcs do, to bring
	* us into refined tangents to the crosstrack. At final correction a PID algorithm is used to maintain fine control.<p/>
	* axis[6] is POV, it has quantized values to represent its states.
	*
	* @param connectedNode
	* @param pubschannel
	* @param message
	* @param twistpub
	* @param twistmsg
	*/
	private void processJoystickMessages(ConnectedNode connectedNode,
			Collection<Publisher<Int32MultiArray>> pubschannel, Joy message, Publisher<Twist> twistpub,
			Twist twistmsg) {
		final geometry_msgs.Vector3 angular = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
		final geometry_msgs.Vector3 linear = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
		final geometry_msgs.Quaternion orientation =  connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE); 
		int axisLED = Integer.parseInt((String) robot.getAXIS()[robot.getLUN("LEDDriver")].get("Axis"));
		int axisBoom = Integer.parseInt((String) robot.getAXIS()[robot.getLUN("BoomActuator")].get("AxisY"));
		int axisLift = Integer.parseInt((String) robot.getAXIS()[robot.getLUN("LiftActuator")].get("Axis"));
		// value of POV pad when actuating for up
		float axisLiftUp = Float.parseFloat((String) robot.getAXIS()[robot.getLUN("LiftActuator")].get("AxisUp"));
		float axisLiftDown = Float.parseFloat((String)robot.getAXIS()[robot.getLUN("LiftActuator")].get("AxisDown"));
		float[] axes = message.getAxes();
		int[] buttons = message.getButtons();

		// check for emergency stop, on X or A or green or lower button
		if( buttons[6] != 0 ) {
			if(DEBUG)
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
		// LEDCameraIlluminatorControl:4
		// LEDCameraIlluminatorSlot:1
		// LEDCameraIlluminatorChannel:1
		//-------------------
		if(DEBUG)
			System.out.printf("%s message:%s vals:%f %f %f %n",this.getClass().getName(), message.toString(), axes[axisLED],axes[axisBoom],axes[axisLift]);
		if(axes[axisLED] != -1 || 
				isActive[robot.getLUN("LEDDriver")]) {
			if(axes[axisLED] == -1) {
				if( isActive[robot.getLUN("LEDDriver")]) {
					ArrayList<Integer> triggerVals = new ArrayList<Integer>(2);
					triggerVals.add(robot.getLUN("LEDDriver"));
					triggerVals.add(0);
					if(DEBUG)
						System.out.println(" turning off LED");
					getPublisher(pubschannel,"LEDDriver").publish(setupPub(connectedNode, triggerVals));
					try {
						Thread.sleep(5);
					} catch (InterruptedException e) {}
				}
				isActive[robot.getLUN("LEDDriver")] = false;
			} else {
				isActive[robot.getLUN("LEDDriver")] = true;
				ArrayList<Integer> triggerVals = new ArrayList<Integer>(2);
				triggerVals.add(robot.getLUN("LEDDriver"));
				triggerVals.add(Integer.valueOf((int)axes[axisLED])*1000);
				getPublisher(pubschannel, "LEDDriver").publish(setupPub(connectedNode, triggerVals));
				try {
					Thread.sleep(5);
				} catch (InterruptedException e) {}
			}
		}
		//---
		// Process right stick (joystick axes [1] right stick x, [3] right stick y)
		// axes[3] is y, value is -1 to 0 to 1, and for some reason forward is negative on the stick
		// scale it from -1 to 0 to 1 to -1000 to 0 to 1000, or the value of SPEEDSCALE which is our speed control range 
		//
		// set it up to send down the publishing pipeline
		//
		if(axes[axisBoom] != 0 ||
			isActive[robot.getLUN("BoomActuator")]) {
			// was active, no longer
			if(axes[axisBoom] == 0) {
				isActive[robot.getLUN("BoomActuator")] = false;
				ArrayList<Integer> speedVals = new ArrayList<Integer>(2);
				speedVals.add(robot.getLUN("BoomActuator")); //controller slot
				speedVals.add(0);
				getPublisher(pubschannel, "BoomActuator").publish(setupPub(connectedNode, speedVals));
				try {
					Thread.sleep(5);
				} catch (InterruptedException e) {}
			} else {
				// may not have been active, now is
				isActive[robot.getLUN("BoomActuator")] = true;
				ArrayList<Integer> speedVals = new ArrayList<Integer>(2);
				speedVals.add(robot.getLUN("BoomActuator")); //controller slot
				speedVals.add((int)(axes[axisBoom])*1000);
				getPublisher(pubschannel, "BoomActuator").publish(setupPub(connectedNode, speedVals));
				try {
					Thread.sleep(5);
				} catch (InterruptedException e) {}
			}
		}
		//----
		// Map the POV values to actions
		//AXIS[4].AxisType:POV
		//AXIS[4].Axis:6
		//AXIS[4].AxisUp:0.25
		//AXIS[4].AxisDown:0.75
		if(axes[axisLift] == axisLiftUp) {
			// set it up to send down the publishing pipeline
			ArrayList<Integer> povVals = new ArrayList<Integer>(2);
			povVals.add(robot.getLUN("LiftActuator"));
			povVals.add(1000);
			getPublisher(pubschannel, "LiftActuator").publish(setupPub(connectedNode, povVals));
			isActive[robot.getLUN("LiftActuator")] = true;
			try {
				Thread.sleep(5);
			} catch (InterruptedException e) {}
		} else {
			if(axes[axisLift] == axisLiftDown) {
				//
				// set it up to send down the publishing pipeline 
				//
				ArrayList<Integer> povVals = new ArrayList<Integer>(2);
				povVals.add(robot.getLUN("LiftActuator")); //controller slot
				povVals.add(-1000);
				getPublisher(pubschannel, "LiftActuator").publish(setupPub(connectedNode, povVals));
				isActive[robot.getLUN("LiftActuator")] = true;
				try {
					Thread.sleep(5);
				} catch (InterruptedException e) {}
			} else {
				if( isActive[robot.getLUN("LiftActuator")]) {
					isActive[robot.getLUN("LiftActuator")]= false;
					ArrayList<Integer> povVals = new ArrayList<Integer>(2);
					povVals.add(robot.getLUN("LiftActuator")); //controller slot
					povVals.add(0);
					getPublisher(pubschannel, "LiftActuator").publish(setupPub(connectedNode, povVals));
					try {
						Thread.sleep(5);
					} catch (InterruptedException e) {}
				}
			}
		}
		/*	
		 if (axes[6] == Component.POV.OFF) {
			 if(DEBUG)System.out.println("POV OFF");
		 } else 
			 if ( axes[6] == Component.POV.UP) {
				 if(DEBUG)System.out.println("POV Up");
			 } else 
				 if ( axes[6] == Component.POV.UP_RIGHT) {
					 if(DEBUG)System.out.println("POV Up_Right");
				 } else 
					 if ( axes[6] == Component.POV.RIGHT) {
						 if(DEBUG)System.out.println("POV Right");
					 } else 
						 if ( axes[6] == Component.POV.DOWN_RIGHT) {
							 if(DEBUG)System.out.println("POV Down Right");
						 } else 
							 if ( axes[6] == Component.POV.DOWN) {
								 if(DEBUG)System.out.println("POV Down");
							 } else 
								 if ( axes[6] == Component.POV.DOWN_LEFT) {
									 if(DEBUG)System.out.println("POV Down left");
								 } else 
									 if ( axes[6] == Component.POV.LEFT) {
										 if(DEBUG)System.out.println("POV Left");
									 } else 
										 if ( axes[6] == Component.POV.UP_LEFT) {
											 
										 }
		*/
		
	}
	
	protected Publisher<Int32MultiArray> getPublisher(Collection<Publisher<Int32MultiArray>> pubschannel, String val) {
		if(DEBUG) {
			System.out.printf("%s %s%n",this.getClass().getName(), val);
		}
		return pubschannel.stream().filter(e -> e.getTopicName().toString().contains(val)).findFirst().get();
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
		PeripheralController mc = new PeripheralController();
		ArrayList<String> pubs = new ArrayList<String>();
		for(NodeDeviceDemuxer ndd : mc.listNodeDeviceDemuxer) {
			System.out.println("/"+ndd.getDeviceName());
			if(DEBUG)
				System.out.println("Bringing up publisher /"+ndd.getDeviceName());
			pubs.add("/"+ndd.getDeviceName());
		}
		System.out.println("----");
		System.out.println(pubs.stream().filter(e -> e.toString().contains("LEDDriver"))
		.findFirst().get());
		System.out.println(pubs.stream().filter(e -> e.toString().contains("LEDDriver"))
				.findFirst().get());

		System.out.println(pubs.stream().filter(e -> e.toString().contains("LEDDriver"))
				.findFirst().get());
	}
}
