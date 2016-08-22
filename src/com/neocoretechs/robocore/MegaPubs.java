package com.neocoretechs.robocore;

import java.io.File;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CountDownLatch;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.message.MessageListener;
import org.ros.message.Time;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.BatteryListener;
import com.neocoretechs.robocore.machine.bridge.MotorFaultListener;
import com.neocoretechs.robocore.machine.bridge.UltrasonicListener;

/**
 * Publish the data acquired from the Mega board through the serial interface. Motor controller, ultrasonic sensor
 * voltage, etc and all that is acquired from the attached USB of an aux board such as Mega2560 via RS-232.
 * As input from the board is received the AsynchDemuxer decodes the header and prepares the data for publication
 * to the Ros bus. If the 'M2' code is issued the data from the controller is enabled.
 * The ByteSerialDataPort and its associates read the base level data organized as a header and followed by each successive line
 * of parameter number and value.
 * 
 * Subscribe to messages on the cmd_vel standard ROS topic to receive motor control commands as TWIST messages and
 * send them on to the controller.
 * 
 * Publish various warnings over the 'robocore/status' topic.
 * 
 * Essentially this is the main interface to the attached Mega2560 and on to all GPIO
 * and high level motor control functions which are activated via a serial board TTL to RS-232 attached to Mega2560
 * UART aux port or low level motor driver H bridge which is issued a series of M codes to activate the
 * PWM and GPIO direction pins on the Mega. 
 * Controller:
 * G5 C<channel> P<power> - Issue move command to controller on channel C (1 or 2 for left/right wheel) with P<-1000 to 1000>
 * a negative value on power means reverse.
 * M2 - start reception of controller messages - fault/ battery/status demultiplexed in AsynchDemuxer
 * H Bridge Driver:
 * M45 P<pin> S<0-255> - start PWM on pin P with value S, many optional arguments for timer setup
 * M41 P<pin> - Set digital pin P high forward
 * M42 P<pin> - Set digital pin P low reverse
 * @author jg
 */
public class MegaPubs extends AbstractNodeMain  {
	private static final boolean DEBUG = true;
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
		return GraphName.of("pubs_robocore");
	}

/**
 * Create NodeConfiguration 
 * @throws URISyntaxException 
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
		connectedNode.newPublisher("range/ultrasonic/robocore", sensor_msgs.Range._TYPE);

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
			System.out.println("Could not start process to read attached serial port.."+e);
			e.printStackTrace();
			return;
		}
	} else {
		AsynchDemuxer.getInstance();	
	}
	
	// set the proper instance of motor control host
	// do we have a smart controller or simple H bridge
	// If the M2 command has been issued at startup in AsynchDemuxer assume a smart controller
	if( AsynchDemuxer.isController) {
		motorControlHost = new MotorControl();
	} else {
		motorControlHost = new MotorControlPWM();
	}
	
	final Subscriber<geometry_msgs.Twist> substwist = 
			connectedNode.newSubscriber("cmd_vel", geometry_msgs.Twist._TYPE);
	
	final Subscriber<std_msgs.UInt32MultiArray> subspwm = 
			connectedNode.newSubscriber("cmd_pwm", std_msgs.UInt32MultiArray._TYPE);
	
	final Subscriber<std_msgs.UInt32MultiArray> subsgpio = 
			connectedNode.newSubscriber("cmd_gpio", std_msgs.UInt32MultiArray._TYPE);
	
	// subscribe to image tag input for emergency stop signal
	final Subscriber<geometry_msgs.Quaternion> tagsub = 
			connectedNode.newSubscriber("ardrone/image_tag", geometry_msgs.Quaternion._TYPE);
	
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
		int targetPitch = (int) val.getX();
		int targetDist = (int) val.getY();
		val = message.getAngular();
		float targetYaw = (float) val.getZ();

		if( targetPitch == 0.0 && targetYaw == 0.0 )
				isMoving = false;
		else
				isMoving = true;
		if( DEBUG )
			System.out.println("Robot commanded to move LIN:" + targetPitch + " ANG:" + targetYaw);
		//log.debug("Robot commanded to move:" + targetPitch + "mm linear in orientation " + targetYaw);
		try {
			if( shouldMove ) {
				int[] speed = motorControlHost.setMotorSpeed(targetPitch, targetYaw);//.moveRobotRelative(targetYaw, targetPitch, targetDist);
				motorControlHost.updateSpeed(speed[0], speed[1]);
			} else
				System.out.println("Emergency stop directive in effect, no move to "+targetDist + "mm, yawANGZ " + targetYaw+" pitchLINX:"+targetPitch);
		} catch (IOException e) {
			System.out.println("there was a problem communicating with motor controller:"+e);
			e.printStackTrace();
		}
	}
	});
	/*
	 * Provide an emergency stop via image recognition facility
	 */
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
	
	subspwm.addMessageListener(new MessageListener<std_msgs.UInt32MultiArray>() {
		@Override
		public void onNewMessage(std_msgs.UInt32MultiArray message) {
			//if( DEBUG )
				System.out.println("Aux directive:"+message.getData());
			if( auxPWM == null )
				auxPWM = new AuxPWMControl();
			try {
				auxPWM.activateAux(message.getData());
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}			
		}
	});
	
	subsgpio.addMessageListener(new MessageListener<std_msgs.UInt32MultiArray>() {
		@Override
		public void onNewMessage(std_msgs.UInt32MultiArray message) {
			//if( DEBUG )
				System.out.println("Aux directive:"+message.getData());
			if( auxGPIO == null )
				auxGPIO = new AuxGPIOControl();
			try {
				auxGPIO.activateAux(message.getData());
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}			
		}
	});
	// tell the waiting constructors that we have registered publishers
	awaitStart.countDown();

	// This CancellableLoop will be canceled automatically when the node shuts
	// down.
	connectedNode.executeCancellableLoop(new CancellableLoop() {
		private int sequenceNumber;
		@Override
		protected void setup() {
			sequenceNumber = 0;
		}

		@Override
		protected void loop() throws InterruptedException {
			boolean upSeq = false;
			std_msgs.Header ihead = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);

			diagnostic_msgs.DiagnosticStatus statmsg = null;
			sensor_msgs.Range rangemsg = null;

				if( !BatteryListener.data.isEmpty() ) {
					Float batt = BatteryListener.data.takeFirst();
					volts = batt.floatValue();
					statmsg = statpub.newMessage();
					statmsg.setName("battery");
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.WARN);
					statmsg.setMessage("Battery voltage warning "+((int)volts)+" volts");
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					statpub.publish(statmsg);
					Thread.sleep(1);
					if( DEBUG) System.out.println("Published seq#"+sequenceNumber+" battery: "+statmsg.getMessage().toString());
				}		
	
				if( !UltrasonicListener.data.isEmpty() ) {
					Integer range = UltrasonicListener.data.takeFirst();
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
					upSeq = true;
				}				
			
	
				if( !MotorFaultListener.data.isEmpty() ) {
					String mfd = MotorFaultListener.data.takeFirst();
					statmsg = statpub.newMessage();
					statmsg.setName("motor");
					statmsg.setLevel(diagnostic_msgs.DiagnosticStatus.ERROR);
					statmsg.setMessage("Motor fault warning "+mfd);
					diagnostic_msgs.KeyValue kv = connectedNode.getTopicMessageFactory().newFromType(diagnostic_msgs.KeyValue._TYPE);
					List<diagnostic_msgs.KeyValue> li = new ArrayList<diagnostic_msgs.KeyValue>();
					li.add(kv);
					statmsg.setValues(li);
					statpub.publish(statmsg);
					if( DEBUG) System.out.println("Published seq#"+sequenceNumber+" motor fault: "+statmsg.getMessage().toString());
				}			
		
			if( upSeq )
				++sequenceNumber;
			Thread.sleep(5);
		}
	});  


}

}
