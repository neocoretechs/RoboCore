package com.neocoretechs.robocore;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;

import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeMainExecutor;
import org.ros.node.service.CountDownServiceServerListener;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.node.topic.SubscriberListener;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.internal.node.topic.PublisherIdentifier;
import org.ros.message.MessageListener;

import sensor_msgs.Range;
import std_msgs.Int32MultiArray;

import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.MarlinspikeControl;
import com.neocoretechs.robocore.marlinspike.MarlinspikeControlInterface;
import com.neocoretechs.robocore.marlinspike.MarlinspikeManager;
import com.neocoretechs.robocore.marlinspike.NodeDeviceDemuxer;
import com.neocoretechs.robocore.marlinspike.PublishDiagnosticResponse;
import com.neocoretechs.robocore.marlinspike.PublishResponseInterface;

import com.neocoretechs.robocore.navigation.NavListenerMotorControlInterface;
import com.neocoretechs.robocore.config.DeviceEntry;
import com.neocoretechs.robocore.config.Robot;
import com.neocoretechs.robocore.config.RobotInterface;
import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;
import com.neocoretechs.robocore.services.ControllerStatusMessage;
import com.neocoretechs.robocore.services.ControllerStatusMessageRequest;
import com.neocoretechs.robocore.services.ControllerStatusMessageResponse;
import com.neocoretechs.robocore.services.GPIOControlMessage;
import com.neocoretechs.robocore.services.GPIOControlMessageRequest;
import com.neocoretechs.robocore.services.GPIOControlMessageResponse;
import com.neocoretechs.robocore.services.PWMControlMessage;
import com.neocoretechs.robocore.services.PWMControlMessageRequest;
import com.neocoretechs.robocore.services.PWMControlMessageResponse;

import diagnostic_msgs.DiagnosticStatus;

/**
 * Process the serial interface data acquired from the Mega board. Real-time telemetry from the Marlinspike realtime subsystem
 * such Motor controller status, ultrasonic sensor, voltage reading, etc and all that is acquired from the attached UART/USB of an aux 
 * board such as Mega2560 via asynchronous bidirectional serial protocol such as RS-232. Since serial ports are monolithic and atomic
 * the actual communications are centralized in the {@code MegaControl} class and its contained {@code ByteSerialDataPort} low level
 * access module.<p/>
 * Certain command line remappings are used to affect behavior as follows:<br/>
 * determine debugging directives __debug:=publisher,demuxer,marlinspike any of these 3 arguments are optional to turn on debugging<br/>
 * __pwm:=controller<br/>
 * Indicates that PWM directives sent as a service are to be processed through a PWM based controller
 * initialized with a previous M-code<p/>
 * __pwm:=direct<br/>
 * Indicates that PWM directives sent as a service are to be directly applied as a set of values working
 * on a PWM pin<p/>
 * GPIO service invocation always works directly on a pin.<p/>  
 * 
 * As input from the attached board is received, the {@code AsynchDemuxer} decodes the header and prepares the data for publication
 * to the Ros bus. Publish various warnings over the 'robocore/status' topic.<p/>
 * <h3>AsynchDemuxer:</h3>
 * The ByteSerialDataPort and its associates read the base level data coming over the USB port from the marlinspike organized 
 * as a header delimited by chevrons and containing the message topic identifier.<p/>
 * The identifier is used to demux the message and properly read the remainder and process it after each successive line.
 * Each line after the header has a parameter number and value. For instance, for an analog pin input or an ultrasonic reading
 * we have '1 pin', '2 reading' with pin and reading being the numeric values for the distance or pin reading value.
 * The end of the message is delimited with '</topic>' with topic being the matching header element.<p/>
 * 
 * Essentially this is the main interface to the attached Mega2560 and on to all GPIO
 * and high level motor control functions which are activated via a serial board TTL to RS-232 attached to Mega2560.
 * Anything attached to the microcontroller: UART aux port or low level motor driver H bridge, issued a series of M codes 
 * to activate the functions of the PWM and GPIO pins on the Mega. 
 * Controller M and G code examples:
 * <code> G5 Z<slot> C<channel> P<power> </code>- Issue move command to controller in slot Z<slot> on channel C (1 or 2 for left/right wheel) with P<-1000 to 1000>
 * a negative value on power means reverse.
 * M2 - start reception of controller messages - fault/ battery/status demultiplexed in AsynchDemuxer
 * H Bridge Driver:
 * <code><br/>
 * M45 P<pin> S<0-255> - start PWM on pin P with value S, many optional arguments for timer setup<br/>
 * M41 P<pin> - Set digital pin P high forward <br/>
 * M42 P<pin> - Set digital pin P low reverse <br/>
 * </code>
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 */
public class MegaPubs extends AbstractNodeMain  {
	private static boolean DEBUG = false;
	Object statMutex = new Object(); 
	Object navMutex = new Object();
	private String host;
	private InetSocketAddress master;
	private CountDownLatch awaitStart = new CountDownLatch(1);
	//private MarlinspikeControlInterface motorControlHost;
	NavListenerMotorControlInterface navListener = null;
	private AuxGPIOControl auxGPIO = null;
	private AuxPWMControl auxPWM = null;
	private boolean isMoving = false;
	private boolean shouldMove = true;
	private int targetPitch;
	private int targetDist;
	private float targetYaw;
	// if angularMode is true, interpret the last TWIST subscription as intended target to move to and
	// compute incoming channel velocities as scaling factors to that movement
	protected boolean angularMode = false;
	private final String RPT_SERVICE = "cmd_report";
	private final String PWM_SERVICE = "cmd_pwm";
	private final String GPIO_SERVICE = "cmd_gpio";
	private String PWM_MODE = "direct"; // in direct mode, our 2 PWM values are pin, value, otherwise values of channel 1 and 2 of slot 0 controller
	// Queue for outgoing diagnostic messages
	CircularBlockingDeque<diagnostic_msgs.DiagnosticStatus> outgoingDiagnostics = new CircularBlockingDeque<diagnostic_msgs.DiagnosticStatus>(256);
	// Queue for outgoing ranging messages, if any such sensors are present
	CircularBlockingDeque<sensor_msgs.Range> outgoingRanges = new CircularBlockingDeque<sensor_msgs.Range>(256);
	// Preload the collection of response handlers that match responses from attached Marlinspike microcontrollers
	String[] stopics = new String[] {
	AsynchDemuxer.topicNames.BATTERY.val(),
	AsynchDemuxer.topicNames.MOTORFAULT.val(), 
	AsynchDemuxer.topicNames.TIME.val(),
	AsynchDemuxer.topicNames.CONTROLLERSTATUS.val(),
	AsynchDemuxer.topicNames.STATUS.val(),
	AsynchDemuxer.topicNames.CONTROLLERSTOPPED.val(),
	AsynchDemuxer.topicNames.NOMORGCODE.val(),
	AsynchDemuxer.topicNames.BADMOTOR.val(),
	AsynchDemuxer.topicNames.BADPWM.val(),
	AsynchDemuxer.topicNames.UNKNOWNG.val(),
	AsynchDemuxer.topicNames.UNKNOWNM.val(),
	AsynchDemuxer.topicNames.BADCONTROL.val(),
	AsynchDemuxer.topicNames.NOCHECKSUM.val(),
	AsynchDemuxer.topicNames.NOLINECHECK.val(),
	AsynchDemuxer.topicNames.CHECKMISMATCH.val(),
	AsynchDemuxer.topicNames.LINESEQ.val(),
	AsynchDemuxer.topicNames.M115.val(),
	AsynchDemuxer.topicNames.ASSIGNEDPINS.val(),
	AsynchDemuxer.topicNames.MOTORCONTROLSETTING.val(),
	AsynchDemuxer.topicNames.PWMCONTROLSETTING.val(),
	AsynchDemuxer.topicNames.DIGITALPINSETTING.val(),
	AsynchDemuxer.topicNames.ANALOGPINSETTING.val(),
	AsynchDemuxer.topicNames.ULTRASONICPINSETTING.val(),
	AsynchDemuxer.topicNames.PWMPINSETTING.val()
	};

	Byte[] publishStatus = new Byte[] {	 diagnostic_msgs.DiagnosticStatus.WARN,
	diagnostic_msgs.DiagnosticStatus.ERROR,
	diagnostic_msgs.DiagnosticStatus.OK,
	diagnostic_msgs.DiagnosticStatus.OK,
	diagnostic_msgs.DiagnosticStatus.OK,
	diagnostic_msgs.DiagnosticStatus.WARN,
	diagnostic_msgs.DiagnosticStatus.ERROR,
	diagnostic_msgs.DiagnosticStatus.ERROR,
	diagnostic_msgs.DiagnosticStatus.ERROR,
	diagnostic_msgs.DiagnosticStatus.ERROR,
	diagnostic_msgs.DiagnosticStatus.ERROR,
	diagnostic_msgs.DiagnosticStatus.ERROR,
	diagnostic_msgs.DiagnosticStatus.ERROR,
	diagnostic_msgs.DiagnosticStatus.ERROR,
	diagnostic_msgs.DiagnosticStatus.ERROR,
	diagnostic_msgs.DiagnosticStatus.ERROR,
	diagnostic_msgs.DiagnosticStatus.OK,
	diagnostic_msgs.DiagnosticStatus.OK,
	diagnostic_msgs.DiagnosticStatus.OK,
	diagnostic_msgs.DiagnosticStatus.OK,
	diagnostic_msgs.DiagnosticStatus.OK,
	diagnostic_msgs.DiagnosticStatus.OK,
	diagnostic_msgs.DiagnosticStatus.OK,
	diagnostic_msgs.DiagnosticStatus.OK};
		
	//
	// Initialize various types of responses that will be published to the various outgoing message busses.
	// Generate base {@link Robot} that bootstraps configs from RoboCore.properties via VM command line directive
	//
	static RobotInterface robot;
	static {
		try {
			robot = new Robot();
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}
	//final AsynchDemuxer asynchDemuxer = new AsynchDemuxer();
	MarlinspikeManager marlinspikeManager = null;
	// Collection of subscriber to node device name
	HashMap<Subscriber<Int32MultiArray>, String> subscriberDevice = new HashMap<Subscriber<Int32MultiArray>, String>();
	HashMap<String, Boolean> isOperating = new HashMap<String, Boolean>();
	
	PublishResponseInterface<diagnostic_msgs.DiagnosticStatus>[] responses;
	PublishResponseInterface<sensor_msgs.Range>[] ultrasonic;
	
	private Collection<String> statPub = Collections.synchronizedCollection(new ArrayList<String>());
	
	public MegaPubs(String[] args) {
		CommandLineLoader cl = new CommandLineLoader(Arrays.asList(args));
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, cl.build());
	}
	
	/**
	 * @throws IOException 
	 */
	public MegaPubs() {

	}
	
	public GraphName getDefaultNodeName() {
		if(DEBUG)
			System.out.printf("Robot reports host name as %s%n",robot.getHostName());
		return GraphName.of(robot.getHostName());
	}

@Override
public void onStart(final ConnectedNode connectedNode) {
	// check command line remappings for __mode:=startup to issue the startup code to the attached processor
	Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
	// determine debugging directives __debug:=publisher,demuxer,marlinspike
	if( remaps.containsKey("__debug") ) {
		String debug = remaps.get("__debug");
		String[] debugs = debug.split(",");
		for(String debugx : debugs) {
			if(debugx.equals("publisher"))
				DEBUG = true;
			if(debugx.equals("demuxer"))
				AsynchDemuxer.DEBUG = true;
			if(debugx.equals("marlinspike"))
				MarlinspikeControl.DEBUG = true;
		}
	}
	// in direct mode, our 2 PWM values are pin, value, otherwise values of channel 1 and 2 of slot 0 controller
	if( remaps.containsKey("__pwm") )
		PWM_MODE = remaps.get("__pwm");

	final Publisher<diagnostic_msgs.DiagnosticStatus> statpub = connectedNode.newPublisher("robocore/status", diagnostic_msgs.DiagnosticStatus._TYPE);
	
	configureMarlinspikeManager(connectedNode, statpub);
	
	//final RosoutLogger log = (Log) connectedNode.getLog();

	final Publisher<sensor_msgs.Range> rangepub = connectedNode.newPublisher("LowerFront/sensor_msgs/Range", sensor_msgs.Range._TYPE);

	// We use the twist topic to get the generic universal stop command for all attached devices when pose = -1,-1,-1
	final Subscriber<geometry_msgs.Twist> substwist = connectedNode.newSubscriber("cmd_vel", geometry_msgs.Twist._TYPE);
	//
	// Iterate the list of device demuxxers we put together in configureMarlinspikeManager.
	// for each NodeDeviceDemuxer, create a subscriber on the channel <demuxer device name> of type Int32MultiArray
	//
	for(DeviceEntry ndd : marlinspikeManager.getDevices()) {
		Subscriber<std_msgs.Int32MultiArray> subscr = connectedNode.newSubscriber(ndd.getName(), std_msgs.Int32MultiArray._TYPE);
		subscriberDevice.put(subscr, ndd.getName());
		configureSubscriberListener(subscr, connectedNode, statpub);
	}
	
	NodeDeviceDemuxer demux = (NodeDeviceDemuxer) marlinspikeManager.getNodeDeviceDemuxers().toArray()[0];
	// Initialize the collection of DiagnosticStatus response handlers
	for(int i = 0; i < stopics.length; i++) {
		responses[i] = new PublishDiagnosticResponse(connectedNode, statpub, outgoingDiagnostics);
		responses[i].takeBridgeAndQueueMessage(stopics[i], demux.getAsynchDemuxer().getTopic(stopics[i]), publishStatus[i]);
	}
	//final Subscriber<std_msgs.Int32MultiArray> subsvelocity = 
	//		connectedNode.newSubscriber("absolute/cmd_vel", std_msgs.Int32MultiArray._TYPE);
	//final Subscriber<std_msgs.Int32MultiArray> substrigger = 
	//connectedNode.newSubscriber("absolute/cmd_periph1", std_msgs.Int32MultiArray._TYPE);
	
	final CountDownServiceServerListener<ControllerStatusMessageRequest, ControllerStatusMessageResponse> serviceServerListener =
	        CountDownServiceServerListener.newDefault();
	configureControllerStatusServer(serviceServerListener, connectedNode, statpub);	

	final CountDownServiceServerListener<PWMControlMessageRequest, PWMControlMessageResponse> servicePWMServerListener =
	        CountDownServiceServerListener.newDefault();
	configurePWMServer(servicePWMServerListener, connectedNode, statpub);
	
	final CountDownServiceServerListener<GPIOControlMessageRequest, GPIOControlMessageResponse> serviceGPIOServerListener =
	        CountDownServiceServerListener.newDefault();
	configureGPIOServer(serviceGPIOServerListener, connectedNode, statpub);	
	
	//Subscriber<sensor_msgs.Range> subsrange = connectedNode.newSubscriber("LowerFront/sensor_msgs/Range", sensor_msgs.Range._TYPE);
	//Subscriber<sensor_msgs.Range> subsrange2 = connectedNode.newSubscriber("UpperFront/sensor_msgs/Range", sensor_msgs.Range._TYPE);

	/**
	 * Extract the linear and angular components from cmd_vel topic Twist quaternion, take the linear X (pitch) and
	 * angular Z (yaw) and send them to motor control. This results in motion planning computing a turn which
	 * involves rotation about a point in space located at a distance related to speed and angular velocity. The distance
	 * is used to compute the radius of the arc segment traversed by the wheel track to make the turn. If not moving we can
	 * make the distance 0 and rotate about a point in space, otherwise we must inscribe an appropriate arc. The distance
	 * in that case is the linear travel, otherwise the distance is the diameter of the arc segment.
	 * If we get commands on the cmd_vel topic we assume we are moving, if we do not get the corresponding IMU readings, we have a problem
	 * If we get a 0,0 on the X,yaw move we stop. If we dont see stable IMU again we have a problem, Houston.
	 * 'targetDist' is x. if x = 0 we are going to turn in place.
	 *  If we are turning in place and th < 0, turn left. if th >= 0 turn right
	 *  If x != 0 and th = 0 its a forward or backward motion
	 *  If x != 0 and th != 0 its a rotation around a point in space with forward motion, describing an arc.
	 *  theta is th and gets calculated as difference of last imu and wheel theta.
	 * // Rotation about a point in space
	 *		if( th < 0 ) { // left
	 *			spd_left = (float) (x - th * wheelTrack / 2.0);
	 *			spd_right = (float) (x + th * wheelTrack / 2.0);
	 *		} else {
	 *			spd_right = (float) (x - th * wheelTrack / 2.0);
	 *			spd_left = (float) (x + th * wheelTrack / 2.0);
	 *		}
	 */
	substwist.addMessageListener(new MessageListener<geometry_msgs.Twist>() {
		@Override
		public void onNewMessage(geometry_msgs.Twist message) {
			geometry_msgs.Vector3 val = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
			val = message.getLinear();
			targetPitch = (int) val.getX();
			targetDist = (int) val.getY();
			val = message.getAngular();
			targetYaw = (float) val.getZ();
			if( targetPitch == -1 && targetDist == -1 && targetYaw == -1) {
				shouldMove = false;
				try {
					for(NodeDeviceDemuxer ndd : marlinspikeManager.getNodeDeviceDemuxers())
						ndd.getMarlinspikeControl().commandStop();
				} catch (IOException e) {
					e.printStackTrace();
				}
				isOperating.replaceAll((key, value) -> false);
			}
				
		}
	});

	/*
	 * Provide an emergency stop via image recognition facility
	 
	tagsub.addMessageListener(new MessageListener<geometry_msgs.Quaternion>() {
		@Override
		public void onNewMessage(geometry_msgs.Quaternion message) {
			try {
				//if( DEBUG )
					System.out.println("Emergency directive:"+message.getW());
				if( (message.getW() >= 0.0 && message.getW() <= 180.0) ) {
					motorControlHost.commandStop();
					isMoving = false;
					shouldMove = false;
					System.out.println("Command stop issued for "+message.getW());
				} else {
					shouldMove = true;
					System.out.println("Movement reinstated for "+message.getW());
				}
			} catch (IOException e) {
				System.out.println("Cannot issue motor stop! "+e);
				e.printStackTrace();
			}
		}
	});
	*/

	/*
	subsrange.addMessageListener(new MessageListener<sensor_msgs.Range>() {
		@Override
		public void onNewMessage(Range message) {
			float range = message.getRange();
			if( DEBUG )
				System.out.println("Floor Range "+range);
			try {
				if(speak && (message.getRange() < 300.0) ) {
					//speaker.doSpeak
					diagnostic_msgs.DiagnosticStatus statmsg = null;
					statmsg = statpub.newMessage();
					statmsg.setName("FloorRange");
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.WARN);
					statmsg.setMessage(message.getRange()++" centimeters from feet");
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					outgoingDiagnostics.addLast(statmsg);
				}
			} catch (Throwable e) {
				e.printStackTrace();
			}	
		}
	});
	
	subsrange2.addMessageListener(new MessageListener<sensor_msgs.Range>() {
		@Override
		public void onNewMessage(Range message) {
			float range = message.getRange();
			if( DEBUG )
				System.out.println("Head Range "+range);
			try {
				if(speak && (message.getRange() < 350.0) ) {
					//speaker.doSpeak
					diagnostic_msgs.DiagnosticStatus statmsg = null;
					statmsg = statpub.newMessage();
					statmsg.setName("HeadRange");
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.WARN);
					statmsg.setMessage(message.getRange()++" centimeters from head");
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					outgoingDiagnostics.addLast(statmsg);
				}
			} catch (Throwable e) {
				e.printStackTrace();
			}	
		}
	});
	*/
	for(Subscriber<std_msgs.Int32MultiArray> subs : subscriberDevice.keySet()) {
		subs.addMessageListener(new MessageListener<std_msgs.Int32MultiArray>() {
			@Override
			public void onNewMessage(std_msgs.Int32MultiArray message) {
				//std_msgs.Int32 valch1 = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Int32._TYPE);
				String deviceName = subscriberDevice.get(subs);
				int[] valch = message.getData();
				if(DEBUG)
					System.out.printf("%s Subscriber:%s DeviceName=%s Message:%s args:%d Thread:%d%n", this.getClass().getName(), subs, deviceName, message.toString(), valch.length, Thread.currentThread().getId());
				for(int iarg = 0; iarg < valch.length; iarg++) {
					int affectorSpeed = valch[iarg];
					if(DEBUG)
						System.out.printf("%s Message:%s DeviceName=%s arg=%d speed:%d operating:%b%n", this.getClass().getName(), message.toString(), 
								deviceName, iarg, affectorSpeed, isOperating.get(deviceName));
					try {
						MarlinspikeControlInterface control = marlinspikeManager.getMarlinspikeControl(deviceName);
						if(DEBUG)
							System.out.printf("%s got Control %s from MarlinspikeManager%n", this.getClass().getName(),control);
						if(control == null) {
							System.out.println("Controller:"+deviceName+" not configured for this node");
							synchronized(statPub) {
								statPub.add("Controller:"+deviceName+" not configured for this node");
								new PublishDiagnosticResponse(connectedNode, statpub, outgoingDiagnostics, subs.toString(), 
										diagnostic_msgs.DiagnosticStatus.ERROR, statPub);
							}
							continue;
						}
						// keep Marlinspike from getting bombed with zeroes
						if(isOperating.get(deviceName)) {
							if(affectorSpeed == 0) {
								control.setDeviceLevel(deviceName, 0);
								isOperating.replace(deviceName, true, false);
							} else {
								control.setDeviceLevel(deviceName, affectorSpeed);
							}
						} else {
							if(affectorSpeed != 0) {
								isOperating.replace(deviceName, false, true);
								control.setDeviceLevel(deviceName, affectorSpeed);
							}
						}
						if(DEBUG)
							System.out.printf("NewMessage, thread %d received Affector directives DeviceName:%s Value:%d%n",Thread.currentThread().getId(),deviceName,affectorSpeed);
					} catch (IOException e) {
						System.out.println("There was a problem communicating with the controller:"+e);
						e.printStackTrace();
						synchronized(statPub) {
							statPub.add("There was a problem communicating with the controller:");
							statPub.addAll(Arrays.stream(e.getStackTrace()).map(m->m.toString()).collect(Collectors.toList()));
							new PublishDiagnosticResponse(connectedNode, statpub, outgoingDiagnostics, subs.toString(), 
								diagnostic_msgs.DiagnosticStatus.ERROR, statPub );
						}
					}
				}
			}
		});
	}
	// tell the waiting constructors that we have registered publishers
	awaitStart.countDown();

	/**
	 * A CancellableLoop will be canceled automatically when the node shuts down.
	 * Check the various topic message queues for entries and if any are present, multiplex them onto the outbound bus
	 * under the DiagnosticStatus tops for any waiting subscribers to process and deal with.
	 */
	connectedNode.executeCancellableLoop(new CancellableLoop() {
		@Override
		protected void setup() {
		}

		@Override
		protected void loop() throws InterruptedException {
		    try {
				awaitStart.await();
			} catch (InterruptedException e) {}

			diagnostic_msgs.DiagnosticStatus statmsg = null;
			sensor_msgs.Range rangemsg = null;	
			
			//ThreadPoolManager.init(stopics);
			// Invoke the collection of response handlers, this is done for each asynchDemuxer attached to this node, i.e. each Marlinspike
		
			for(int i = 0; i < stopics.length; i++) {
				responses[i].publish();
			}
			//ThreadPoolManager.getInstance().spin(ultrasonic, "SYSTEM");
			//
			// Poll the outgoing message array for diagnostics enqueued by the above processing
			//
			while(!outgoingDiagnostics.isEmpty()) {
				statmsg = outgoingDiagnostics.takeFirst();
				statpub.publish(statmsg);
				if(DEBUG)
					System.out.println("Published "+statmsg.getMessage());
				Thread.sleep(1);
			}
			//
			// Poll the outgoing message array for ranges enqueued by the above processing
			// Ultrasonic messages SHOULD trigger the range message
			//
			while(!outgoingRanges.isEmpty()) {
				rangemsg = outgoingRanges.takeFirst();
				rangepub.publish(rangemsg);
				if(DEBUG)
					System.out.println("Published "+rangemsg.getRange());
				Thread.sleep(1);
			}
					
			Thread.sleep(1);
		}
	});  

}
	/**
	 * Configure the GPIO direct service.
	 * @param serviceGPIOServerListener
	 * @param connectedNode
	 * @param tstatpub
	 */
	private void configureGPIOServer(CountDownServiceServerListener<GPIOControlMessageRequest, GPIOControlMessageResponse> serviceGPIOServerListener,
		ConnectedNode connectedNode, Publisher<DiagnosticStatus> tstatpub) {
		final ServiceServer<GPIOControlMessageRequest, GPIOControlMessageResponse> serviceGPIOServer = connectedNode.newServiceServer(GPIO_SERVICE, GPIOControlMessage._TYPE,
			    new ServiceResponseBuilder<GPIOControlMessageRequest, GPIOControlMessageResponse>() {
					@Override
					public void build(GPIOControlMessageRequest request, GPIOControlMessageResponse response) {	
						//try {
							System.out.println("GPIO direct");
							if( auxGPIO == null )
								auxGPIO = new AuxGPIOControl();
							auxGPIO.activateAux(marlinspikeManager.getMarlinspikeControl("GPIO"), request.getData().getData());
							response.setData("success");
						/*} catch (NoSuchElementException e) {
							System.out.println("EXCEPTION ACTIVATING MARLINSPIKE VIA GPIO SERVICE");
							e.printStackTrace();
							response.setData("fail");
							synchronized(statPub) {
								statPub.add("EXCEPTION ACTIVATING MARLINSPIKE VIA GPIO SERVICE:"+e);
								new PublishDiagnosticResponse(connectedNode, statpub, outgoingDiagnostics, "MARLINSPIKE GPIO ACTIVATION", 
										diagnostic_msgs.DiagnosticStatus.ERROR, statPub );
								while(!outgoingDiagnostics.isEmpty()) {
									try {
										statpub.publish(outgoingDiagnostics.takeFirst());
										Thread.sleep(1);
									} catch (InterruptedException e1) {}
								}
							}
						}*/
					}
				});	
			serviceGPIOServer.addListener(serviceGPIOServerListener);	      
			try {
				serviceGPIOServerListener.awaitMasterRegistrationSuccess(1, TimeUnit.SECONDS);
			} catch (InterruptedException e1) {
				System.out.println("GPIO SERVICE REGISTRATION WAS INTERRUPTED");
				e1.printStackTrace();
			}
	
	}
	/**
	 * Configure the PWM direct service
	 * @param servicePWMServerListener
	 * @param connectedNode
	 * @param tstatpub
	 */
	private void configurePWMServer(CountDownServiceServerListener<PWMControlMessageRequest, PWMControlMessageResponse> servicePWMServerListener,
		ConnectedNode connectedNode, Publisher<DiagnosticStatus> tstatpub) {
		final ServiceServer<PWMControlMessageRequest, PWMControlMessageResponse> servicePWMServer = connectedNode.newServiceServer(PWM_SERVICE, PWMControlMessage._TYPE,
			    new ServiceResponseBuilder<PWMControlMessageRequest, PWMControlMessageResponse>() {
					@Override
					public void build(PWMControlMessageRequest request, PWMControlMessageResponse response) {	
						try {
							switch(PWM_MODE) {
								case "controller":
									break;
									// Directly Activates PWM M45 Pin Level on Marlinspike with 2 values of request.
									// Activates a direct PWM timer without going through one of the various controller
									// objects. Issues an M45 to the Marlinspike on the Mega2560
								case "direct":
								default:
									System.out.println("PWM direct");
									if( auxPWM == null )
										auxPWM = new AuxPWMControl();
									auxPWM.activateAux(marlinspikeManager.getMarlinspikeControl("PWM"), request.getData().getData());	
									break;
							}
							response.setData("success");
						} catch (IOException e) {
							System.out.println("EXCEPTION ACTIVATING MARLINSPIKE VIA PWM SERVICE");
							e.printStackTrace();
							response.setData("fail");
							synchronized(statPub) {
								statPub.add("EXCEPTION ACTIVATING MARLINSPIKE VIA PWM SERVICE:"+e);
								new PublishDiagnosticResponse(connectedNode, tstatpub, outgoingDiagnostics, "MARLINSPIKE PWM ACTIVATION", 
										diagnostic_msgs.DiagnosticStatus.ERROR, statPub );
								while(!outgoingDiagnostics.isEmpty()) {
									try {
										tstatpub.publish(outgoingDiagnostics.takeFirst());
										Thread.sleep(1);
									} catch (InterruptedException e1) {}
								}
							}
						}
					}
				});	
			servicePWMServer.addListener(servicePWMServerListener);	      
			try {
				servicePWMServerListener.awaitMasterRegistrationSuccess(1, TimeUnit.SECONDS);
			} catch (InterruptedException e1) {
				System.out.println("PWM SERVICE REGISTRATION WAS INTERRUPTED");
				e1.printStackTrace();
			}
	}
	/**
	 * Configure the controller status service
	 * @param serviceServerListener
	 * @param connectedNode
	 * @param tstatpub
	 */
	private void configureControllerStatusServer(CountDownServiceServerListener<ControllerStatusMessageRequest, ControllerStatusMessageResponse> serviceServerListener, 
			ConnectedNode connectedNode, Publisher<DiagnosticStatus> tstatpub) {
		final ServiceServer<ControllerStatusMessageRequest, ControllerStatusMessageResponse> serviceServer = connectedNode.newServiceServer(RPT_SERVICE, ControllerStatusMessage._TYPE,
		    new ServiceResponseBuilder<ControllerStatusMessageRequest, ControllerStatusMessageResponse>() {
				@Override
				public void build(ControllerStatusMessageRequest request,ControllerStatusMessageResponse response) {
					StringBuilder sb = new StringBuilder();
					try {
						switch(request.getData()) {
							case "id":
								for(NodeDeviceDemuxer ndd : marlinspikeManager.getNodeDeviceDemuxers())
									sb.append(ndd.getMarlinspikeControl().reportSystemId());
								response.setData(sb.toString());
								break;
							case "reset":
								for(NodeDeviceDemuxer ndd : marlinspikeManager.getNodeDeviceDemuxers())
									sb.append(ndd.getMarlinspikeControl().commandReset());
								response.setData(sb.toString());
								break;
							case "status":
							default:
								for(NodeDeviceDemuxer ndd : marlinspikeManager.getNodeDeviceDemuxers())
									sb.append(ndd.getMarlinspikeControl().reportAllControllerStatus());
								response.setData(sb.toString());
								break;		
						}
					} catch (IOException e) {
						System.out.println("EXCEPTION FROM SERVICE REQUESTING ALL CONTROLLER STATUS REPORT FROM MARLINSPIKE:"+e);
						e.printStackTrace();
						synchronized(statPub) {
							statPub.add("EXCEPTION FROM SERVICE REQUESTING ALL CONTROLLER STATUS REPORT FROM MARLINSPIKE:"+e);
							new PublishDiagnosticResponse(connectedNode, tstatpub, outgoingDiagnostics, "MARLINSPIKE STATUS", 
									diagnostic_msgs.DiagnosticStatus.ERROR, statPub );
							while(!outgoingDiagnostics.isEmpty()) {
								try {
									tstatpub.publish(outgoingDiagnostics.takeFirst());
									Thread.sleep(1);
								} catch (InterruptedException e1) {}
							}
						}
					}
				}
			});	
		serviceServer.addListener(serviceServerListener);	      
		try {
			serviceServerListener.awaitMasterRegistrationSuccess(1, TimeUnit.SECONDS);
		} catch (InterruptedException e1) {
			System.out.println("REPORT SERVICE REGISTRATION WAS INTERRUPTED");
			e1.printStackTrace();
		}
	}

	/**
	* Configure a subscriber to talk to a MarlinSpikeControlInterface which receives 
	* a message of type: LUN#, value of control for that LUN#. <br/>
	* Process the commands from remote source to extract 32 bit int values and apply those to the
	* device channels LUNs. The ordinals represent a logical unit LUN in the configuration which is
	* a collection of node, device, slot, channel of a controller attached to a ROS node, which has
	* a number of tty ports (devices) which have marlinspike boards attached (Mega2560 via USB running the firmware)
	* which have a number of logical software controllers configured by slot and channels.<p/>
	* Process a trigger value from the remote source, most likely a controller such as PS/3.<p/>
	* Take the 2 trigger values as int32 and send them on to the microcontroller 
	* related subsystem at the int valued LUNs. The marlinspike subsystem is composed of a software 
	* controller instance talking to a hardware driver such as an H-bridge or half bridge or even a simple switch.
	* @param subscr The target subscriber
	* @param connectedNode the Ros node we are using
	* @param tstatpub the diagnostic channel if error, called through creation of new PublishDiagnosticResponse
	*/
	private void configureSubscriberListener(Subscriber<Int32MultiArray> subscr, ConnectedNode connectedNode, 
					Publisher<DiagnosticStatus> tstatpub) {
		subscr.addSubscriberListener(new SubscriberListener<std_msgs.Int32MultiArray>() {
				@Override
				public void onNewPublisher(Subscriber<Int32MultiArray> subs, PublisherIdentifier pubs) {
					if(DEBUG)
						System.out.printf("%s Subscsriber %s new publisher %s%n", this.getClass().getName(), subs, pubs);				
				}
				@Override
				public void onShutdown(Subscriber<std_msgs.Int32MultiArray> subs) {
					if(DEBUG)
						System.out.printf("%s Subscsriber %s SHUTDOWN%n", this.getClass().getName(), subs);	
				}
				@Override
				public void onMasterRegistrationFailure(Subscriber<Int32MultiArray> subs) {
					if(DEBUG)
						System.out.printf("%s Subscsriber %s failed to register with master!%n", this.getClass().getName(), subs);				
				}
				@Override
				public void onMasterRegistrationSuccess(Subscriber<Int32MultiArray> subs) {
					if(DEBUG)
						System.out.printf("%s Subscsriber %s successfully registered with master%n", this.getClass().getName(), subs);
					//final int affector = ((ArrayList<Subscriber<Int32MultiArray>>)subschannel).indexOf(subs);
				}
				@Override
				public void onMasterUnregistrationFailure(Subscriber<Int32MultiArray> subs) {			
					if(DEBUG)
						System.out.printf("%s Subscsriber %s failed to unregister with master!%n", this.getClass().getName(), subs);	
				}
				@Override
				public void onMasterUnregistrationSuccess(Subscriber<Int32MultiArray> subs) {
					if(DEBUG)
						System.out.printf("%s Subscsriber %s failed to register with master!%n", this.getClass().getName(), subs);	
				}
		});
	}

	/**
	 * Create the MarlinspikeManager and the list of diagnostic responses to publish based on the stopics collection.
	 * If there is a problem we will try to publish exception to diagnostic channel if possible and throw a RuntimeException.<p/>
	 * Here, we configure the critical listNodeDeviceDemuxer that is used in the onStart method above.
	 * @param connectedNode The Ros node we are currently initializing
	 * @param statpub the diagnostic status publisher channel
	 */
	private void configureMarlinspikeManager(ConnectedNode connectedNode, Publisher<DiagnosticStatus> tstatpub) {
		// TODO get this from parameter server or singleton with map of robot names
		try {
			marlinspikeManager = new MarlinspikeManager(robot);
			marlinspikeManager.configureMarlinspike(false, true);
			// the collection of NodeDeviceDemuxer will be accumulated based on the node name entries in the properties file,
			// if it matched the name of this host the entry is included in the collection. 
			// In this way only entries that apply to Marlinspikes attached to this host are utilized.
			for(DeviceEntry ndd: marlinspikeManager.getDevices()) {
				if(DEBUG)
					System.out.printf("%s Setting DeviceName %s to non-operational status%n", this.getClass().getName(), ndd.getName());	
				isOperating.put(ndd.getName(), false);
			}
			responses = new PublishDiagnosticResponse[stopics.length];
		} catch (IOException e) {
			System.out.println("Could not connect to Marlinspike.."+e);
			e.printStackTrace();
			synchronized(statPub) {
				statPub.add("Could not connect to Marlinspike.."+e);
				new PublishDiagnosticResponse(connectedNode, tstatpub, outgoingDiagnostics, "MARLINSPIKE CONNECTION", 
					diagnostic_msgs.DiagnosticStatus.ERROR, statPub );
				while(!outgoingDiagnostics.isEmpty()) {
					try {
						tstatpub.publish(outgoingDiagnostics.takeFirst());
						Thread.sleep(1);
					} catch (InterruptedException e1) {}
				}
			}
			throw new RuntimeException(e);
		}
	}
	
	
	public static void main(String[] args) {
		
	}
}
