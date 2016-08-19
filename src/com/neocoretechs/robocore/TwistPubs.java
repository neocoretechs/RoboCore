package com.neocoretechs.robocore;

import geometry_msgs.Twist;

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
import org.ros.internal.loader.CommandLineLoader;

import com.neocoretechs.robocore.machine.bridge.AnalogPinListener;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.DigitalPinListener;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.MotorFaultListener;

/**
 * Publishes user constructed Twist messages on the cmd_vel topic. Takes the demuxxed values
 * from the attached device that collects 2 axis data from joystick etc and translates to X linear and Z
 * angular values
 * @author jg
 */
public class TwistPubs extends AbstractNodeMain  {
	private static final boolean DEBUG = true;
	
	float volts;
	float rangetop, rangebot;
	
	Object statMutex = new Object(); 
	Object navMutex = new Object();
	Object rngMutex1 = new Object();
	Object rngMutex2 = new Object();
	Object rngbot = new Object();
	Object rngtop = new Object();
	
	private String host;
	private InetSocketAddress master;
	private CountDownLatch awaitStart = new CountDownLatch(1);
	geometry_msgs.Twist twistmsg = null;
	//private static int[] data = new int[]{-10,10,100};

	final static int joystickMin = 0;
	final static int joystickMax = 1024;
	final static int motorSpeedMax = 1000;

	// scale it to motor speed
	final static int sf = motorSpeedMax / (joystickMax/2);
	
	
	
	public TwistPubs(String host, InetSocketAddress master) {
		this.host = host;
		this.master = master;
		NodeConfiguration nc = build();
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, nc);
	    try {
			awaitStart.await();
		} catch (InterruptedException e) {}
	    
	}
	
	public TwistPubs(String[] args) {
		CommandLineLoader cl = new CommandLineLoader(Arrays.asList(args));
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, cl.build());
	    try {
			awaitStart.await();
		} catch (InterruptedException e) {}
	}
	
	public TwistPubs() {}
	
	public GraphName getDefaultNodeName() {
		return GraphName.of("pubs_twist");
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
	
	final Publisher<geometry_msgs.Twist> twistpub =
		connectedNode.newPublisher("cmd_vel", geometry_msgs.Twist._TYPE);
	//final geometry_msgs.Vector3 angular = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
	//final geometry_msgs.Vector3 linear = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
	//final geometry_msgs.Quaternion orientation =  connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE); 
	
	//Subscriber<sensor_msgs.Imu> subsimu = connectedNode.newSubscriber("ardrone/navdata", sensor_msgs.Imu._TYPE);
	Subscriber<sensor_msgs.Range> subsrangetop = connectedNode.newSubscriber("range/ultrasonic/ardrone", sensor_msgs.Range._TYPE);
	Subscriber<sensor_msgs.Range> subsrangebot = connectedNode.newSubscriber("range/ultrasonic/robocore", sensor_msgs.Range._TYPE);
	// Start reading from serial port
	// check command line remappings for __mode:=startup to issue the startup code to the attached processor
	// ONLY DO IT ONCE ON INIT!
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
	/*
	subsimu.addMessageListener(new MessageListener<sensor_msgs.Imu>() {
		@Override
		public void onNewMessage(sensor_msgs.Imu message) {
			synchronized(navMutex) {
				try {
				angular.setX(message.getAngularVelocity().getX());
				angular.setY(message.getAngularVelocity().getY());
				angular.setZ(message.getAngularVelocity().getZ());
				linear.setX(message.getLinearAcceleration().getX());
				linear.setY(message.getLinearAcceleration().getY());
				linear.setZ(message.getLinearAcceleration().getZ());
				orientation.setX(message.getOrientation().getX());
				orientation.setY(message.getOrientation().getY());
				orientation.setZ(message.getOrientation().getZ());
				orientation.setW(message.getOrientation().getW());
				//if( DEBUG )
				//	System.out.println("Got nav message:"+message);
				} catch (Throwable e) {
					System.out.println("Nav subs exception:"+e.getMessage());
					e.printStackTrace();
				}
			}
		}

		});
		*/
		subsrangetop.addMessageListener(new MessageListener<sensor_msgs.Range>() {
		@Override
		public void onNewMessage(sensor_msgs.Range message) {
			synchronized(rngMutex1) {
				try {
					rangetop = message.getRange();
				} catch (Throwable e) {
					System.out.println("Range top subs exception:"+e.getMessage());
					e.printStackTrace();
				}
			}
		}
		});

		subsrangebot.addMessageListener(new MessageListener<sensor_msgs.Range>() {
		@Override
		public void onNewMessage(sensor_msgs.Range message) {
			synchronized(rngMutex2) {
				try {
					rangebot = message.getRange();
				} catch (Throwable e) {
					System.out.println("Range bottom subs exception:"+e.getMessage());
					e.printStackTrace();
				}
			}
		}
		});
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
			// look for digital pin override to pivot left/right
			if( !DigitalPinListener.data.isEmpty()) {
				int[] mr = DigitalPinListener.data.peekFirst();
				if( mr[0] == DigitalPinListener.leftPivotPin) { 
					DigitalPinListener.data.takeFirst();
					if( twistmsg == null )
						twistmsg = twistpub.newMessage();
					geometry_msgs.Vector3 val = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
					//val.setZ(-motorSpeedMax/2);
					val.setZ(32767);
					twistmsg.setAngular(val);
					geometry_msgs.Vector3 valx = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
					valx.setX(0);
					twistmsg.setLinear(valx);	
				} else {
					if( mr[0] == DigitalPinListener.rightPivotPin) {
						DigitalPinListener.data.takeFirst();
						if( twistmsg == null )
							twistmsg = twistpub.newMessage();
						geometry_msgs.Vector3 val = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
						//val.setZ(motorSpeedMax/2);
						val.setZ(-32767);
						twistmsg.setAngular(val);
						geometry_msgs.Vector3 valx = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
						valx.setX(0);
						twistmsg.setLinear(valx);	
					} else {
						if( mr[0] == DigitalPinListener.stopPin) {
							DigitalPinListener.data.takeFirst();
							if( twistmsg == null )
								twistmsg = twistpub.newMessage();
							geometry_msgs.Vector3 val = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
							val.setZ(0);
							twistmsg.setAngular(val);
							geometry_msgs.Vector3 valx = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
							valx.setX(0);
							twistmsg.setLinear(valx);
						}
					}
				}
			}
			
			//synchronized(data) {
			//for(int i = 0; i < 2; i++) {
			if( !AnalogPinListener.data.isEmpty() ) {
				int[] mr = AnalogPinListener.data.peekFirst();
				// wait for our pins to be displayed as we are potentially demuxxing numerous analogpin messages
				// look for Y input (throttle)
				if( mr[0] == AnalogPinListener.joystickPinY) {// linear pin
					AnalogPinListener.data.takeFirst();
					geometry_msgs.Vector3 val = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
					int r = mr[1];
					r = (r - (joystickMax/2)) * -1;	
					r *= sf;
					// should be set to +- joystick scaled to motor max +-
					val.setX(r);
					val.setY(0);
					val.setZ(0);
					if( twistmsg == null )
						twistmsg = twistpub.newMessage();
					twistmsg.setLinear(val);
					if( DEBUG) System.out.println("Set twist linear "+val.getX()+" raw:"+mr[1]);
				}
				// look for x input (pivot)
				if( mr[0] == AnalogPinListener.joystickPinX) {// angular pin
					AnalogPinListener.data.takeFirst();
					geometry_msgs.Vector3 val = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
					int r = mr[1];
					r = (r - (joystickMax/2));
					// scale it to motor speed
					r *= sf;
					// should be set to +- joystick scaled to motor max +-
					val.setZ(r);
					val.setX(0);
					val.setY(0);
					if( twistmsg == null )
						twistmsg = twistpub.newMessage();
					twistmsg.setAngular(val);
					if( DEBUG) System.out.println("Set twist angular "+val.getZ()+" raw:"+mr[1]);
				}
			}
			//}
			// If we have a full set of coords publish, if we only get a partial then one of them
			// we in the deadband but that should not preclude a publication of valid coord plus deadband center
			// thus leaving no dead stick
			if( twistmsg != null && twistmsg.getLinear() != null && twistmsg.getAngular() != null) {
				twistpub.publish(twistmsg);
				if( DEBUG ) {
					System.out.println("Published:"+twistmsg.getLinear().getX()+","+twistmsg.getAngular().getZ());
					//printGyro(linear, angular, orientation);
				}
				twistmsg = null;
			} else {
				if( twistmsg != null && twistmsg.getLinear() != null) {
					geometry_msgs.Vector3 val = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
					val.setZ((joystickMax/2));
					twistmsg.setAngular(val);
					twistpub.publish(twistmsg);
					if( DEBUG ) {
						System.out.println("Published fillin angular:"+twistmsg.getLinear().getX()+","+twistmsg.getAngular().getZ());
						//printGyro(linear, angular, orientation);
					}
					twistmsg = null;
				} /*else {
					if( twistmsg != null && twistmsg.getAngular() != null) {
						geometry_msgs.Vector3 val = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
						val.setX((joystickMax/2));
						twistmsg.setLinear(val);
						twistpub.publish(twistmsg);
						if( DEBUG ) {
							System.out.println("Published fillin linear:"+twistmsg.getLinear().getX()+","+twistmsg.getAngular().getZ());
							//printGyro(linear, angular, orientation);
						}
						twistmsg = null;
					}
				}*/
			}
				//mc.moveRobotRelative(.5f, 45, 100);
				//mc.moveRobotRelative(-.25f, 45, 100);
				//mc.moveRobotRelative(.25f, 45, 100);
				//mc.moveRobotRelative(-.25f, 225, 100);
				//mc.moveRobotRelative(.0f, 0, 100);
				//val = message.getLinear();
				//int targetPitch = (int) val.getX();
				//int targetDist = (int) val.getY();
				//val = message.getAngular();
				//float targetYaw = (float) val.getZ();
				//motorControlHost.moveRobotRelative(targetYaw, targetPitch, targetDist);
	
			Thread.sleep(10);
		}
	});  


}

private void printGyro(geometry_msgs.Vector3 angular, geometry_msgs.Vector3 linear, geometry_msgs.Quaternion orientation) {
	synchronized(navMutex) {
		System.out.println("Angular x:"+angular.getX());
		System.out.println("Angular y:"+angular.getY());
		System.out.println("Angular z:"+angular.getZ());
		System.out.println("Linear x:"+linear.getX());
		System.out.println("Linear y:"+linear.getY());
		System.out.println("Linear z:"+linear.getZ());
		System.out.println("Gyro x:"+orientation.getX());
		System.out.println("Gyro y:"+orientation.getY());
		System.out.println("gyro z:"+orientation.getZ());
		System.out.println("gyro w:"+orientation.getW());
		System.out.println("Range top"+rangetop+" bottom:"+rangebot);
	}
}

}
