package com.neocoretechs.robocore;

import java.io.File;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.text.DateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
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
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.message.MessageListener;
import org.ros.message.Time;

import rosgraph_msgs.Log;
import sensor_msgs.Range;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.TopicListInterface;
import com.neocoretechs.robocore.marlinspike.PublishDiagnosticResponse;
import com.neocoretechs.robocore.marlinspike.PublishResponseInterface;
import com.neocoretechs.robocore.marlinspike.PublishResponses;
import com.neocoretechs.robocore.marlinspike.PublishUltrasonicResponse;
import com.neocoretechs.robocore.navigation.NavListenerMotorControlInterface;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
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

import diagnostic_msgs.DiagnosticStatus;

/**
 * Process the serial interface data acquired from the Mega board. Real-time telemetry from the Marlinspike realtime subsystem
 * such Motor controller status, ultrasonic sensor, voltage reading, etc and all that is acquired from the attached UART/USB of an aux 
 * board such as Mega2560 via asynchronous bidirectional serial protocol such as RS-232. Since serial ports are monolithic and atomic
 * the actual communications are centralized in the {@code MegaControl} class and its contained {@code ByteSerialDataPort} low level
 * access module.
 * 
 * The command line parameters drive the process through the following:<p/>
 * __mode:=startup<br/>
 * Indicates that the M and G code file specified, default startup.gcode, read a set of initialization codes
 * to be sent to the Marlinspike<p/>
 * __pwm:=controller<br/>
 * Indicates that PWM directives sent as a service are to be processed through a PWM based controller
 * initialized with a previous M-code<p/>
 * __pwm:=direct<br/>
 * Indicates that PWM directives sent as a service are to be directly applied as a set of values working
 * on a PWM pin<p/>
 * GPIO service invocation always works directly on a pin.<p/>  
 * In addition, control functions are 
 * As input from the attached board is received, the {@code AsynchDemuxer} decodes the header and prepares the data for publication
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
 * The cmd_gpio message is also processed activating a specific pin with a specific value as a service
 * mentioned above.
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
	private boolean[] isOperating = {false,false,false};
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
	// Queue for outgoing range messages
	CircularBlockingDeque<sensor_msgs.Range> outgoingRanges = new CircularBlockingDeque<sensor_msgs.Range>(256);
	// Preload the collection of response handlers
	String[] stopics = new String[] {AsynchDemuxer.topicNames.BATTERY.val(),
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
	AsynchDemuxer.topicNames.PWMPINSETTING.val()};

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
	PublishResponseInterface<diagnostic_msgs.DiagnosticStatus>[] responses = new PublishDiagnosticResponse[stopics.length];
	PublishResponseInterface<sensor_msgs.Range> ultrasonic;
	
	public MegaPubs(String host, InetSocketAddress master) {
		this.host = host;
		this.master = master;
		NodeConfiguration nc = build();
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, nc);   
	}
	
	public MegaPubs(String[] args) {
		CommandLineLoader cl = new CommandLineLoader(Arrays.asList(args));
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, cl.build());
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
	final AsynchDemuxer asynchDemuxer = new AsynchDemuxer();
	try {
		asynchDemuxer.connect(ByteSerialDataPort.getInstance());
		asynchDemuxer.init();
		motorControlHost = new MegaControl(asynchDemuxer);
	} catch (IOException e) {
		System.out.println("Could not connect to Marlinspike.."+e);
		e.printStackTrace();
		return;
	}
	
	//final RosoutLogger log = (Log) connectedNode.getLog();

	final Publisher<diagnostic_msgs.DiagnosticStatus> statpub =
		connectedNode.newPublisher("robocore/status", diagnostic_msgs.DiagnosticStatus._TYPE);

	final Publisher<sensor_msgs.Range> rangepub = 
		connectedNode.newPublisher("LowerFront/sensor_msgs/Range", sensor_msgs.Range._TYPE);

	
	// Start reading from serial port
	// check command line remappings for __mode:=startup to issue the startup code to the attached processor
	Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
	String mode="";
	if( remaps.containsKey("__mode") )
		mode = remaps.get("__mode");
	if( mode.equals("startup")) {
		try {
			asynchDemuxer.config();
		} catch (IOException e) {
			System.out.println("Could not issue configuration commands to Marlinspike.."+e);
			e.printStackTrace();
			return;
		}
	}
	if( remaps.containsKey("__pwm") )
		PWM_MODE = remaps.get("__pwm");

	
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
						switch(request.getData()) {
							case "id":
								response.setData(motorControlHost.reportSystemId());
								break;
							case "reset":
								response.setData(motorControlHost.commandReset());
								break;
							case "status":
							default:
								response.setData(motorControlHost.reportAllControllerStatus());
								break;		
						}
					} catch (IOException e) {
						System.out.println("EXCEPTION FROM SERVICE REQUESTING ALL CONTROLLER STATUS REPORT FROM MARLINSPIKE:"+e);
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
							auxPWM.activateAux(asynchDemuxer, request.getData().getData());	
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
					auxGPIO.activateAux(asynchDemuxer, request.getData().getData());
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
			if( targetPitch == -1 && targetDist == -1 && targetYaw == -1) {
				shouldMove = false;
				try {
					motorControlHost.commandStop();
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
				
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
		if(valch.length != 6)
			return;
		// multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
		for(int i = 0; i < valch.length; i+=6) {
			int valch1 = valch[i]; // slot
			int valch2 = valch[i+1]; // channel
			int valch3 = valch[i+2]; // value
			int valch4 = valch[i+3]; // slot
			int valch5 = valch[i+4]; // channel
			int valch6 = valch[i+5]; // value
			if( valch1 == 0.0 && valch2 == 0.0 ) {
				isMoving = false;
			} else {
				shouldMove = true;
				isMoving = true;
			}
			try {
				if( shouldMove ) {
					if( DEBUG ) {
						System.out.println("Robot commanded to move Left wheel ABS slot "+valch1+" channel:"+valch2+ ":" + valch3+"\r\nRight wheel ABS slot "+valch4+" channel:"+valch5+ ":" + valch6);
					}
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
	 */
	substrigger.addMessageListener(new MessageListener<std_msgs.Int32MultiArray>() {
	@Override
	public void onNewMessage(std_msgs.Int32MultiArray message) {
		int[] valch = message.getData();
		try {
			if( valch.length == 5 ) {
				// lift
				int valch1 = valch[0];
				int valch2 = valch[1];
				int affectorSpeed = valch[3];
				// keep Marlinspike from getting bombed with zeroes
				if(affectorSpeed == 0) {
					if(isOperating[2]) {
						motorControlHost.setAffectorDriveSpeed(valch1, valch2, 0);
					}
					isOperating[2] = false;
				} else {
					isOperating[2] = true;
					if(DEBUG)
						System.out.println("Subs trigger, recieved Affector directives slot:"+valch1+" channel:"+valch2+" value:"+affectorSpeed);
					motorControlHost.setAffectorDriveSpeed(valch1, valch2, affectorSpeed);
				}
			} else {
				if( valch.length == 4 ) {
					// boom
					int valch1 = valch[0];
					int valch2 = valch[1];
					int affectorSpeed = valch[3];
					// keep Marlinspike from getting bombed with zeroes
					if(affectorSpeed == 0) {
						if(isOperating[1]) {
							motorControlHost.setAffectorDriveSpeed(valch1, valch2, 0);
						}
						isOperating[1] = false;
					} else {
						isOperating[1] = true;
						if(DEBUG)
							System.out.println("Subs trigger, recieved Affector directives slot:"+valch1+" channel:"+valch2+" value:"+affectorSpeed);
						motorControlHost.setAffectorDriveSpeed(valch1, valch2, affectorSpeed);
					}
				} else {
					if(valch.length == 3) {
						// LED
						int valch1 = valch[0];
						int valch2 = valch[1];
						int valch3 = valch[2];
						if(valch3 == 0) {
							if(isOperating[0]) {
								((PWMControlInterface)motorControlHost).setAbsolutePWMLevel(valch1, valch2, 0);
							}
							isOperating[0] = false;
						} else {			
							isOperating[0] = true;
							if(DEBUG)
								System.out.println("Subs trigger, recieved PWM directives slot:"+valch1+" channel:"+valch2+" value:"+valch3);
							((PWMControlInterface)motorControlHost).setAbsolutePWMLevel(valch1, valch2, valch3);
						}
					}
				}
			}
		} catch (IOException e) {
						System.out.println("there was a problem communicating with PWM controller:"+e);
						e.printStackTrace();
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

	//ThreadPoolManager.init(stopics);
	// Initialize the collection of DiagnosticStatus response handlers
	for(int i = 0; i < stopics.length; i++) {
		responses[i] = new PublishDiagnosticResponse(asynchDemuxer, connectedNode, statpub, outgoingDiagnostics);
		responses[i].takeBridgeAndQueueMessage(stopics[i], publishStatus[i]);
		//ThreadPoolManager.getInstance().spin(responses[i], stopics[i]);
	}
	// spin individual responders or other groups of responders as necessary
	ultrasonic = new PublishUltrasonicResponse(asynchDemuxer, connectedNode, rangepub, outgoingRanges);
	ultrasonic.takeBridgeAndQueueMessage(AsynchDemuxer.topicNames.ULTRASONIC.val(), DiagnosticStatus.OK);
	//ThreadPoolManager.getInstance().spin(ultrasonic, "SYSTEM");
	
	// tell the waiting constructors that we have registered publishers
	awaitStart.countDown();

	/**
	 * 	A CancellableLoop will be canceled automatically when the node shuts down.
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
			// Invoke the collection of response handlers
			for(int i = 0; i < stopics.length; i++) {
				responses[i].publish();
			}
			// spin individual responders or other groups of responders as necessary
			ultrasonic.publish();
			//ThreadPoolManager.getInstance().spin(ultrasonic, "SYSTEM");
			//
			// Poll the outgoing message array for diagnostics enqueued by the above processing
			//
			while(!outgoingDiagnostics.isEmpty()) {
				statmsg = outgoingDiagnostics.takeFirst();
				statpub.publish(statmsg);
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
				System.out.println("Published "+rangemsg.getRange());
				Thread.sleep(1);
			}
					
			Thread.sleep(1);
		}
	});  

}


}
