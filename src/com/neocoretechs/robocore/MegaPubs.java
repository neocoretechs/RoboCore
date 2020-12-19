package com.neocoretechs.robocore;


import java.io.File;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

import org.ros.concurrent.CancellableLoop;
import org.ros.exception.DuplicateServiceException;
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
import org.ros.node.service.ServiceServerListener;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.internal.node.CountDownRegistrantListener;
import org.ros.message.MessageListener;
import org.ros.message.Time;

import rosgraph_msgs.Log;
import sensor_msgs.Range;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.TopicListInterface;
import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;
import com.neocoretechs.robocore.machine.bridge.MachineBridge;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.propulsion.MotorControlInterface2D;
import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;
import com.neocoretechs.robocore.services.ControllerStatusMessage;
import com.neocoretechs.robocore.services.ControllerStatusMessageRequest;
import com.neocoretechs.robocore.services.ControllerStatusMessageResponse;
import com.neocoretechs.robocore.services.GPIOControlMessage;
import com.neocoretechs.robocore.services.GPIOControlMessageRequest;
import com.neocoretechs.robocore.services.GPIOControlMessageResponse;
import com.neocoretechs.robocore.services.PWMControlMessage;
import com.neocoretechs.robocore.services.PWMControlMessageRequest;
import com.neocoretechs.robocore.services.PWMControlMessageResponse;

/**
 * Publish the data acquired from the Mega board through the serial interface. Motor controller, ultrasonic sensor
 * voltage, etc and all that is acquired from the attached USB of an aux board such as Mega2560 via RS-232.
 * 
 * As input from the attached board is received the AsynchDemuxer decodes the header and prepares the data for publication
 * to the Ros bus. Publish various warnings over the 'robocore/status' topic.
 * The ByteSerialDataPort and its associates read the base level data organized as a header delimited by chevrons and
 * containing the message topic identifier. After the identifier, which is used to demux the message and properly read the
 * remainder and process it after each successive line.
 * Each line after the header has a parameter number and value. For instance, for an analog pin input or an ultrasonic reading
 * we have '1 pin', '2 reading' with pin and reading being the numeric values for the distance or pin reading value.
 * The end of the message is delimited with '</topic>' with topic being the matching header element.
 * 
 * On the subscriber side, Subscribe to messages on the cmd_vel standard ROS topic to receive motor control commands as TWIST messages and
 * send them on to the attached controller. We have the standard cmd_vel which receives TWIST messages and an aux "absolute/cmd_vel"
 * that we use to send absolute motor control acceleration directives, such as from the PS3 controller.
 * We also subscribe to the cmd_pwm message, which allows us to issue a M45 PWM directive for a particular power level.
 * The cmd_gpio message is also processed activating a specific pin with a specific value.
 * 
 * Essentially this is the main interface to the attached Mega2560 and on to all GPIO
 * and high level motor control functions which are activated via a serial board TTL to RS-232 attached to Mega2560.
 * Anything attached to the microcontroller: UART aux port or low level motor driver H bridge, issued a series of M codes 
 * to activate the functions of the PWM and GPIO pins on the Mega. 
 * Controller M and G code examples:
 * G5 Z<slot> C<channel> P<power> - Issue move command to controller in slot Z<slot> on channel C (1 or 2 for left/right wheel) with P<-1000 to 1000>
 * a negative value on power means reverse.
 * M2 - start reception of controller messages - fault/ battery/status demultiplexed in AsynchDemuxer
 * H Bridge Driver:
 * M45 P<pin> S<0-255> - start PWM on pin P with value S, many optional arguments for timer setup
 * M41 P<pin> - Set digital pin P high forward
 * M42 P<pin> - Set digital pin P low reverse
 * @author jg
 */
public class MegaPubs extends AbstractNodeMain  {
	private static final boolean DEBUG = false;
	float volts;
	Object statMutex = new Object(); 
	Object navMutex = new Object();
	private String host;
	private InetSocketAddress master;
	private CountDownLatch awaitStart = new CountDownLatch(1);
	private MotorControlInterface2D motorControlHost;
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
	
	public MegaPubs(String host, InetSocketAddress master) {
		this.host = host;
		this.master = master;
		NodeConfiguration nc = build();
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, nc);
	    try {
			awaitStart.await();
		} catch (InterruptedException e) {}
	    
	}
	
	public MegaPubs(String[] args) {
		CommandLineLoader cl = new CommandLineLoader(Arrays.asList(args));
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, cl.build());
	    try {
			awaitStart.await();
		} catch (InterruptedException e) {}
	}
	
	public MegaPubs() {}
	
	public GraphName getDefaultNodeName() {
		return GraphName.of("megapubs");
	}

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
public void onStart(final ConnectedNode connectedNode) {
	//final RosoutLogger log = (Log) connectedNode.getLog();

	final Publisher<diagnostic_msgs.DiagnosticStatus> statpub =
		connectedNode.newPublisher("robocore/status", diagnostic_msgs.DiagnosticStatus._TYPE);

	final Publisher<sensor_msgs.Range> rangepub = 
		connectedNode.newPublisher("LowerFront/sensor_msgs/Range", sensor_msgs.Range._TYPE);

	try {
		AsynchDemuxer.getInstance().connect();
	} catch (IOException e) {
		System.out.println("Could not connect to Marlinspike.."+e);
		e.printStackTrace();
		return;
	}
	
	// Start reading from serial port
	// check command line remappings for __mode:=startup to issue the startup code to the attached processor
	Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
	String mode="";
	if( remaps.containsKey("__mode") )
		mode = remaps.get("__mode");
	if( mode.equals("startup")) {
		try {
			AsynchDemuxer.getInstance().config();
		} catch (IOException e) {
			System.out.println("Could not issue configuration commands to Marlinspike.."+e);
			e.printStackTrace();
			return;
		}
	}
	if( remaps.containsKey("__pwm") )
		PWM_MODE = remaps.get("__pwm");

	AsynchDemuxer.getInstance().init();

	motorControlHost = new MegaControl();
	
	final Subscriber<geometry_msgs.Twist> substwist = 
			connectedNode.newSubscriber("cmd_vel", geometry_msgs.Twist._TYPE);
	
	final Subscriber<std_msgs.Int32MultiArray> subsvelocity = 
			connectedNode.newSubscriber("absolute/cmd_vel", std_msgs.Int32MultiArray._TYPE);
	
	final Subscriber<std_msgs.Int32MultiArray> substrigger = 
	connectedNode.newSubscriber("absolute/cmd_periph1", std_msgs.Int32MultiArray._TYPE);
	

	final CountDownServiceServerListener<ControllerStatusMessageRequest, ControllerStatusMessageResponse> serviceServerListener =
		        CountDownServiceServerListener.newDefault();
	final ServiceServer<ControllerStatusMessageRequest, ControllerStatusMessageResponse> serviceServer = connectedNode.newServiceServer(RPT_SERVICE, ControllerStatusMessage._TYPE,
		    new ServiceResponseBuilder<ControllerStatusMessageRequest, ControllerStatusMessageResponse>() {
				@Override
				public void build(ControllerStatusMessageRequest request,ControllerStatusMessageResponse response) {	
					try {
						response.setData(motorControlHost.reportAllControllerStatus());
					} catch (IOException e) {
						System.out.println("EXCEPTION ACTIVATING MARLINSPIKE VIA REPORT SERVICE");
						e.printStackTrace();
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

	final CountDownServiceServerListener<PWMControlMessageRequest, PWMControlMessageResponse> servicePWMServerListener =
	        CountDownServiceServerListener.newDefault();
	final ServiceServer<PWMControlMessageRequest, PWMControlMessageResponse> servicePWMServer = connectedNode.newServiceServer(PWM_SERVICE, PWMControlMessage._TYPE,
	    new ServiceResponseBuilder<PWMControlMessageRequest, PWMControlMessageResponse>() {
			@Override
			public void build(PWMControlMessageRequest request, PWMControlMessageResponse response) {	
				try {
					switch(PWM_MODE) {
						case "controller":
							motorControlHost.setAbsolutePWMLevel(0, 1, request.getData().getData()[0], 
									0, 2, request.getData().getData()[1]);
							break;
							// Directly Activates PWM M45 Pin Level on Marlinspike with 2 values of request.
							// Activates a direct PWM timer without going through one of the various controller
							// objects. Issues an M45 to the Marlinspike on the Mega2560
						case "direct":
						default:
							System.out.println("PWM direct");
							if( auxPWM == null )
								auxPWM = new AuxPWMControl();
							auxPWM.activateAux(request.getData().getData());	
							break;
					}
					response.setData("success");
				} catch (IOException e) {
					System.out.println("EXCEPTION ACTIVATING MARLINSPIKE VIA PWM SERVICE");
					e.printStackTrace();
					response.setData("fail");
				}
			}
		});	
	servicePWMServer.addListener(servicePWMServerListener);	      
	try {
		serviceServerListener.awaitMasterRegistrationSuccess(1, TimeUnit.SECONDS);
	} catch (InterruptedException e1) {
		System.out.println("PWM SERVICE REGISTRATION WAS INTERRUPTED");
		e1.printStackTrace();
	}
	
	final CountDownServiceServerListener<GPIOControlMessageRequest, GPIOControlMessageResponse> serviceGPIOServerListener =
	        CountDownServiceServerListener.newDefault();
	final ServiceServer<GPIOControlMessageRequest, GPIOControlMessageResponse> serviceGPIOServer = connectedNode.newServiceServer(GPIO_SERVICE, GPIOControlMessage._TYPE,
	    new ServiceResponseBuilder<GPIOControlMessageRequest, GPIOControlMessageResponse>() {
			@Override
			public void build(GPIOControlMessageRequest request, GPIOControlMessageResponse response) {	
				try {
					System.out.println("GPIO direct");
					if( auxGPIO == null )
						auxGPIO = new AuxGPIOControl();
					auxGPIO.activateAux(request.getData().getData());
					response.setData("success");
				} catch (IOException e) {
					System.out.println("EXCEPTION ACTIVATING MARLINSPIKE VIA GPIO SERVICE");
					e.printStackTrace();
					response.setData("fail");
				}
			}
		});	
	servicePWMServer.addListener(servicePWMServerListener);	      
	try {
		serviceServerListener.awaitMasterRegistrationSuccess(1, TimeUnit.SECONDS);
	} catch (InterruptedException e1) {
		System.out.println("GPIO SERVICE REGISTRATION WAS INTERRUPTED");
		e1.printStackTrace();
	}
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
		}
	});
	/**
	 * Process the direct velocity commands from remote source to extract 2 3x32 bit int values and apply those to the
	 * left and right propulsion wheels. the values represent <slot><channel<value> for each wheel where slot = controller
	 * slot dynamically assigned vie M-code in the MEGA firmware, <channel> is the channel in the controller in the slot,
	 * and value is the value to be applied to the channel in the slot.
	 */
	subsvelocity.addMessageListener(new MessageListener<std_msgs.Int32MultiArray>() {
	@Override
	public void onNewMessage(std_msgs.Int32MultiArray message) {
		//std_msgs.Int32 valch1 = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Int32._TYPE);
		//std_msgs.Int32 valch2 = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Int32._TYPE);
		int[] valch = message.getData();
		// multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
		for(int i = 0; i < valch.length; i+=6) {
			int valch1 = valch[i]; // slot
			int valch2 = valch[i+1]; // channel
			int valch3 = valch[i+2]; // value
			int valch4 = valch[i+3]; // slot
			int valch5 = valch[i+4]; // channel
			int valch6 = valch[i+5]; // value
			if( valch1 == 0.0 && valch2 == 0.0 )
				isMoving = false;
			else
				isMoving = true;
			//if( DEBUG )
				System.out.println("Robot commanded to move Left wheel ABS slot "+valch1+" channel:"+valch2+ ":" + valch3);
				System.out.println("Right wheel ABS slot "+valch4+" channel:"+valch5+ ":" + valch6);

			try {
				if( shouldMove ) {
					//if(angularMode )
					//	motorControlHost.setMotorArcSpeed(valch1, valch2, valch3, valch4, targetPitch, targetYaw);//.moveRobotRelative(targetYaw, targetPitch, targetDist);
					//else
					motorControlHost.setAbsoluteDiffDriveSpeed(valch1, valch2, valch3, valch4, valch5, valch6);
				} else
					System.out.println("Emergency stop directive in effect, no motor power slot:"+valch1+
							" channel:"+valch2+" slot:"+valch4+" channel:"+valch5);
			} catch (IOException e) {
				System.out.println("there was a problem communicating with motor controller:"+e);
				e.printStackTrace();
			}
		}
	}
	});
	/**
	 * Process a trigger value from the remote source, most likely a controller such as PS/3.
	 * Take the 2 trigger values as int32 and send them on to the microcontroller PWM non-propulsion 
	 * related subsystem. this subsystem is composed of a software controller instance talking to a 
	 * hardware driver such as an H-bridge or half bridge or even a simple switch.
	 * the values here are <slot> <channel> <value>.
	 * Alternately, we are sending a -1 as the channel value to invoke emergency stop. 
	 */
	substrigger.addMessageListener(new MessageListener<std_msgs.Int32MultiArray>() {
	@Override
	public void onNewMessage(std_msgs.Int32MultiArray message) {
		int[] valch = message.getData();
		for(int i = 0; i < valch.length; i+=3) {
			int valch1 = valch[i];
			int valch2 = valch[i+1];
			int valch3 = valch[i+2];
			try {
				if(valch1 == -1) {
					motorControlHost.commandStop();
				} else {
					((PWMControlInterface)motorControlHost).setAbsolutePWMLevel(valch1, valch2, valch3);
				}
			} catch (IOException e) {
				System.out.println("there was a problem communicating with motor controller:"+e);
				e.printStackTrace();
			}
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
	
	// tell the waiting constructors that we have registered publishers
	awaitStart.countDown();


	/**
	 * 	A CancellableLoop will be canceled automatically when the node shuts down.
	 * Check the various topic message queues for entries and if any are present, multiplex them onto the outbound bus
	 * under the DiagnosticStatus tops for any waiting subscribers to process and deal with.
	 */
	connectedNode.executeCancellableLoop(new CancellableLoop() {
		private int sequenceNumber;
		@Override
		protected void setup() {
			sequenceNumber = 0;
		}

		@Override
		protected void loop() throws InterruptedException {
			std_msgs.Header ihead = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);

			diagnostic_msgs.DiagnosticStatus statmsg = null;
			sensor_msgs.Range rangemsg = null;
			TopicListInterface tli = AsynchDemuxer.getInstance().getTopic(AsynchDemuxer.topicNames.BATTERY.val());
			MachineBridge mb = tli.getMachineBridge();
			if( !mb.get().isEmpty() ) {
					Float batt = (Float) tli.getResult(mb.waitForNewReading());
					volts = batt.floatValue();
					statmsg = statpub.newMessage();
					statmsg.setName(AsynchDemuxer.topicNames.BATTERY.val());
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.WARN);
					statmsg.setMessage(AsynchDemuxer.topicNames.BATTERY.val()+" voltage warning "+((int)volts)+" volts");
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					outgoingDiagnostics.addFirst(statmsg); // push this top top for priority
					//statpub.publish(statmsg);
					//Thread.sleep(1);
					if( DEBUG) 
						System.out.println("Queued seq#"+sequenceNumber+" "+AsynchDemuxer.topicNames.BATTERY.val()+":"+statmsg.getMessage().toString());
			}		
			tli = AsynchDemuxer.getInstance().getTopic(AsynchDemuxer.topicNames.ULTRASONIC.val());
			mb = tli.getMachineBridge();
			if( !mb.get().isEmpty() ) {
					Integer range = (Integer) tli.getResult(mb.waitForNewReading());
					ihead.setSeq(sequenceNumber);
					Time tst = connectedNode.getCurrentTime();
					ihead.setStamp(tst);
					ihead.setFrameId("0");
					rangemsg = rangepub.newMessage();
					rangemsg.setHeader(ihead);
					rangemsg.setFieldOfView(30);
					rangemsg.setMaxRange(600);
					rangemsg.setMinRange(6);
					rangemsg.setRadiationType(sensor_msgs.Range.ULTRASOUND);
					rangemsg.setRange(range.floatValue());
					rangepub.publish(rangemsg);
					Thread.sleep(1);
					if( DEBUG ) System.out.println("Published seq#"+sequenceNumber+" range: "+rangemsg.getRange());
					++sequenceNumber;
				}				
			tli = AsynchDemuxer.getInstance().getTopic(AsynchDemuxer.topicNames.MOTORFAULT.val());
			mb = tli.getMachineBridge();
			if( !mb.get().isEmpty() ) {
					String mfd = (String) tli.getResult(mb.waitForNewReading());
					statmsg = statpub.newMessage();
					statmsg.setName(AsynchDemuxer.topicNames.MOTORFAULT.val());
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.ERROR);
					statmsg.setMessage(AsynchDemuxer.topicNames.MOTORFAULT.val()+" warning "+mfd);
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					//statpub.publish(statmsg);
					outgoingDiagnostics.addLast(statmsg);
					if( DEBUG) 
						System.out.println("Queued seq#"+sequenceNumber+" "+AsynchDemuxer.topicNames.MOTORFAULT.val()+": "+statmsg.getMessage().toString());
				}
			
			tli = AsynchDemuxer.getInstance().getTopic(AsynchDemuxer.topicNames.TIME.val());
			mb = tli.getMachineBridge();
			if( !mb.get().isEmpty() ) {
					String mfd = (String) tli.getResult(mb.waitForNewReading());
					statmsg = statpub.newMessage();
					statmsg.setName(AsynchDemuxer.topicNames.TIME.val());
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.OK);
					statmsg.setMessage(AsynchDemuxer.topicNames.TIME.val()+" Time "+mfd);
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					outgoingDiagnostics.addLast(statmsg);
					if( DEBUG) 
						System.out.println("Queued seq#"+sequenceNumber+" "+AsynchDemuxer.topicNames.TIME.val()+": "+statmsg.getMessage().toString());
				}
			
			tli = AsynchDemuxer.getInstance().getTopic(AsynchDemuxer.topicNames.CONTROLLERSTATUS.val());
			mb = tli.getMachineBridge();
			if( !mb.get().isEmpty() ) {
					String mfd = (String) tli.getResult(mb.waitForNewReading());
					statmsg = statpub.newMessage();
					statmsg.setName(AsynchDemuxer.topicNames.CONTROLLERSTATUS.val());
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.ERROR);
					statmsg.setMessage(AsynchDemuxer.topicNames.CONTROLLERSTATUS.val()+" warning "+mfd);
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					//statpub.publish(statmsg);
					outgoingDiagnostics.addLast(statmsg);
					if( DEBUG) 
						System.out.println("Queued seq#"+sequenceNumber+" "+AsynchDemuxer.topicNames.CONTROLLERSTATUS.val()+": "+statmsg.getMessage().toString());
				}
			
			tli = AsynchDemuxer.getInstance().getTopic(AsynchDemuxer.topicNames.CONTROLLERSTOPPED.val());
			mb = tli.getMachineBridge();
			if( !mb.get().isEmpty() ) {
					String mfd = (String) tli.getResult(mb.waitForNewReading());
					statmsg = statpub.newMessage();
					statmsg.setName(AsynchDemuxer.topicNames.CONTROLLERSTOPPED.val());
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.ERROR);
					statmsg.setMessage(AsynchDemuxer.topicNames.CONTROLLERSTOPPED.val()+" warning "+mfd);
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					//statpub.publish(statmsg);
					outgoingDiagnostics.addLast(statmsg);
					if( DEBUG) 
						System.out.println("Queued seq#"+sequenceNumber+" "+AsynchDemuxer.topicNames.CONTROLLERSTOPPED.val()+": "+statmsg.getMessage().toString());
				}
			
			tli = AsynchDemuxer.getInstance().getTopic(AsynchDemuxer.topicNames.NOMORGCODE.val());
			mb = tli.getMachineBridge();
			if( !mb.get().isEmpty() ) {
					String mfd = (String) tli.getResult(mb.waitForNewReading());
					statmsg = statpub.newMessage();
					statmsg.setName(AsynchDemuxer.topicNames.NOMORGCODE.val());
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.ERROR);
					statmsg.setMessage(AsynchDemuxer.topicNames.NOMORGCODE.val()+" warning "+mfd);
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					//statpub.publish(statmsg);
					outgoingDiagnostics.addLast(statmsg);
					if( DEBUG) 
						System.out.println("Queued seq#"+sequenceNumber+" "+AsynchDemuxer.topicNames.NOMORGCODE.val()+": "+statmsg.getMessage().toString());
				}	
			
			tli = AsynchDemuxer.getInstance().getTopic(AsynchDemuxer.topicNames.BADMOTOR.val());
			mb = tli.getMachineBridge();
			if( !mb.get().isEmpty() ) {
					String mfd = (String) tli.getResult(mb.waitForNewReading());
					statmsg = statpub.newMessage();
					statmsg.setName(AsynchDemuxer.topicNames.BADMOTOR.val());
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.ERROR);
					statmsg.setMessage(AsynchDemuxer.topicNames.BADMOTOR.val()+" warning "+mfd);
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					//statpub.publish(statmsg);
					outgoingDiagnostics.addLast(statmsg);
					if( DEBUG) 
						System.out.println("Queued seq#"+sequenceNumber+" "+AsynchDemuxer.topicNames.BADMOTOR.val()+": "+statmsg.getMessage().toString());
				}
			
			tli = AsynchDemuxer.getInstance().getTopic(AsynchDemuxer.topicNames.BADPWM.val());
			mb = tli.getMachineBridge();
			if( !mb.get().isEmpty() ) {
					String mfd = (String) tli.getResult(mb.waitForNewReading());
					statmsg = statpub.newMessage();
					statmsg.setName(AsynchDemuxer.topicNames.BADPWM.val());
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.ERROR);
					statmsg.setMessage(AsynchDemuxer.topicNames.BADPWM.val()+" warning "+mfd);
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					//statpub.publish(statmsg);
					outgoingDiagnostics.addLast(statmsg);
					if( DEBUG) 
						System.out.println("Queued seq#"+sequenceNumber+" "+AsynchDemuxer.topicNames.BADPWM.val()+": "+statmsg.getMessage().toString());
				}	
			
			tli = AsynchDemuxer.getInstance().getTopic(AsynchDemuxer.topicNames.UNKNOWNG.val());
			mb = tli.getMachineBridge();
			if( !mb.get().isEmpty() ) {
					String mfd = (String) tli.getResult(mb.waitForNewReading());
					statmsg = statpub.newMessage();
					statmsg.setName(AsynchDemuxer.topicNames.UNKNOWNG.val());
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.ERROR);
					statmsg.setMessage(AsynchDemuxer.topicNames.UNKNOWNG.val()+" warning "+mfd);
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					//statpub.publish(statmsg);
					outgoingDiagnostics.addLast(statmsg);
					if( DEBUG) 
						System.out.println("Queued seq#"+sequenceNumber+" "+AsynchDemuxer.topicNames.UNKNOWNG.val()+": "+statmsg.getMessage().toString());
				}	
			
			tli = AsynchDemuxer.getInstance().getTopic(AsynchDemuxer.topicNames.UNKNOWNM.val());
			mb = tli.getMachineBridge();
			if( !mb.get().isEmpty() ) {
					String mfd = (String) tli.getResult(mb.waitForNewReading());
					statmsg = statpub.newMessage();
					statmsg.setName(AsynchDemuxer.topicNames.UNKNOWNM.val());
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.ERROR);
					statmsg.setMessage(AsynchDemuxer.topicNames.UNKNOWNM.val()+" warning "+mfd);
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					//statpub.publish(statmsg);
					outgoingDiagnostics.addLast(statmsg);
					if( DEBUG) 
						System.out.println("Queued seq#"+sequenceNumber+" "+AsynchDemuxer.topicNames.UNKNOWNM.val()+": "+statmsg.getMessage().toString());
				}
			tli = AsynchDemuxer.getInstance().getTopic(AsynchDemuxer.topicNames.BADCONTROL.val());
			mb = tli.getMachineBridge();
			if( !mb.get().isEmpty() ) {
					String mfd = (String) tli.getResult(mb.waitForNewReading());
					statmsg = statpub.newMessage();
					statmsg.setName(AsynchDemuxer.topicNames.BADCONTROL.val());
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.ERROR);
					statmsg.setMessage(AsynchDemuxer.topicNames.BADCONTROL.val()+" warning "+mfd);
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					//statpub.publish(statmsg);
					outgoingDiagnostics.addLast(statmsg);
					if( DEBUG) 
						System.out.println("Queued seq#"+sequenceNumber+" "+AsynchDemuxer.topicNames.BADCONTROL.val()+": "+statmsg.getMessage().toString());
				}	
			
			tli = AsynchDemuxer.getInstance().getTopic(AsynchDemuxer.topicNames.NOCHECKSUM.val());
			mb = tli.getMachineBridge();
			if( !mb.get().isEmpty() ) {
					String mfd = (String) tli.getResult(mb.waitForNewReading());
					statmsg = statpub.newMessage();
					statmsg.setName(AsynchDemuxer.topicNames.NOCHECKSUM.val());
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.ERROR);
					statmsg.setMessage(AsynchDemuxer.topicNames.NOCHECKSUM.val()+" warning "+mfd);
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					//statpub.publish(statmsg);
					outgoingDiagnostics.addLast(statmsg);
					if( DEBUG) 
						System.out.println("Queued seq#"+sequenceNumber+" "+AsynchDemuxer.topicNames.NOCHECKSUM.val()+": "+statmsg.getMessage().toString());
				}	
			
			tli = AsynchDemuxer.getInstance().getTopic(AsynchDemuxer.topicNames.NOLINECHECK.val());
			mb = tli.getMachineBridge();
			if( !mb.get().isEmpty() ) {
					String mfd = (String) tli.getResult(mb.waitForNewReading());
					statmsg = statpub.newMessage();
					statmsg.setName(AsynchDemuxer.topicNames.NOLINECHECK.val());
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.ERROR);
					statmsg.setMessage(AsynchDemuxer.topicNames.NOLINECHECK.val()+" warning "+mfd);
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					//statpub.publish(statmsg);
					outgoingDiagnostics.addLast(statmsg);
					if( DEBUG) 
						System.out.println("Queued seq#"+sequenceNumber+" "+AsynchDemuxer.topicNames.NOLINECHECK.val()+": "+statmsg.getMessage().toString());
				}
			
			tli = AsynchDemuxer.getInstance().getTopic(AsynchDemuxer.topicNames.CHECKMISMATCH.val());
			mb = tli.getMachineBridge();
			if( !mb.get().isEmpty() ) {
					String mfd = (String) tli.getResult(mb.waitForNewReading());
					statmsg = statpub.newMessage();
					statmsg.setName(AsynchDemuxer.topicNames.CHECKMISMATCH.val());
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.ERROR);
					statmsg.setMessage(AsynchDemuxer.topicNames.CHECKMISMATCH.val()+" warning "+mfd);
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					//statpub.publish(statmsg);
					outgoingDiagnostics.addLast(statmsg);
					if( DEBUG) 
						System.out.println("Queued seq#"+sequenceNumber+" "+AsynchDemuxer.topicNames.CHECKMISMATCH.val()+": "+statmsg.getMessage().toString());
				}
			
			tli = AsynchDemuxer.getInstance().getTopic(AsynchDemuxer.topicNames.LINESEQ.val());
			mb = tli.getMachineBridge();
			if( !mb.get().isEmpty() ) {
					String mfd = (String) tli.getResult(mb.waitForNewReading());
					statmsg = statpub.newMessage();
					statmsg.setName(AsynchDemuxer.topicNames.LINESEQ.val());
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.ERROR);
					statmsg.setMessage(AsynchDemuxer.topicNames.LINESEQ.val()+" warning "+mfd);
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					//statpub.publish(statmsg);
					outgoingDiagnostics.addLast(statmsg);
					if( DEBUG) 
						System.out.println("Queued seq#"+sequenceNumber+" "+AsynchDemuxer.topicNames.LINESEQ.val()+": "+statmsg.getMessage().toString());
				}	
			
			tli = AsynchDemuxer.getInstance().getTopic(AsynchDemuxer.topicNames.M115.val());
			mb = tli.getMachineBridge();
			if( !mb.get().isEmpty() ) {
					String mfd = (String) tli.getResult(mb.waitForNewReading());
					statmsg = statpub.newMessage();
					statmsg.setName(AsynchDemuxer.topicNames.M115.val());
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.ERROR);
					statmsg.setMessage(AsynchDemuxer.topicNames.M115.val()+" warning "+mfd);
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					//statpub.publish(statmsg);
					outgoingDiagnostics.addLast(statmsg);
					if( DEBUG) 
						System.out.println("Queued seq#"+sequenceNumber+" "+AsynchDemuxer.topicNames.M115.val()+": "+statmsg.getMessage().toString());
				}			
			//
			// Poll the outgoing message array for diagnostics enqueued by al the above processing
			//
			while(!outgoingDiagnostics.isEmpty()) {
				statmsg = outgoingDiagnostics.takeFirst();
				statpub.publish(statmsg);
				System.out.println("Published "+statmsg.getMessage());
				++sequenceNumber;
				Thread.sleep(1);
			}
				
			Thread.sleep(10);
		}
	});  

}


}
