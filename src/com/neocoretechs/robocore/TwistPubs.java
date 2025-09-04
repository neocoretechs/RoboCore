package com.neocoretechs.robocore;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Arrays;
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
import org.ros.internal.loader.CommandLineLoader;

import com.neocoretechs.robocore.machine.bridge.TopicListInterface;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.MarlinspikeManager;
import com.neocoretechs.robocore.config.Robot;
import com.neocoretechs.robocore.config.RobotInterface;
import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;
import com.neocoretechs.robocore.machine.bridge.MachineBridge;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;


/**
 * Publishes user constructed Twist messages on the cmd_vel topic. Takes the demuxxed values
 * from the attached device that collects 2 axis data from joystick etc and translates to X linear and Z
 * angular values
 * @author jg
 */
public class TwistPubs extends AbstractNodeMain  {
	private static final boolean DEBUG = false;
	float volts;
	Object statMutex = new Object(); 
	Object navMutex = new Object();
	private String host;
	private InetSocketAddress master;
	private CountDownLatch awaitStart = new CountDownLatch(1);
	geometry_msgs.Twist twistmsg = null;
	// analog inputs on pins 55,56 of Mega2560 as defined in startup.gcode for AsynchDemuxer
	public final static int joystickPinY = 55;
	public final static int joystickPinX = 56;
	//public CircularBlockingDeque<int[]> pubdata = new CircularBlockingDeque<int[]>(16);
	//private static int[] data = new int[]{-10,10,100};

	final static int joystickMin = 0;
	final static int joystickMax = 1024;
	final static int motorSpeedMax = 1000;
	// scale it to motor speed
	final static int sf = motorSpeedMax / (joystickMax/2);
	
	
	public TwistPubs(String[] args) {
		CommandLineLoader cl = new CommandLineLoader(Arrays.asList(args));
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, cl.build());
	}
	
	public TwistPubs() {}
	
	public GraphName getDefaultNodeName() {
		return GraphName.of("pubs_twist");
	}


@Override
public void onStart(final ConnectedNode connectedNode) {
	final AsynchDemuxer asynchDemuxer;
	//fileReader reader = new fileReader();
	//ThreadPoolManager.getInstance().spin(reader, "SYSTEM");
	//final RosoutLogger log = (Log) connectedNode.getLog();
	
	final Publisher<geometry_msgs.Twist> twistpub =
		connectedNode.newPublisher("cmd_vel", geometry_msgs.Twist._TYPE);
	// Start reading from serial port
	// check command line remappings for __mode:=startup to issue the startup code to the attached processor
	// ONLY DO IT ONCE ON INIT!
	//Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
	//String mode="";
	//if( remaps.containsKey("__mode") )
	//	mode = remaps.get("__mode");
	//if( mode.equals("startup")) {
		try {
			RobotInterface robot = new Robot();
			MarlinspikeManager mm = new MarlinspikeManager(robot); 
			asynchDemuxer = new AsynchDemuxer(mm);
			asynchDemuxer.connect(new ByteSerialDataPort());
			AsynchDemuxer.config(asynchDemuxer);
		} catch (IOException e) {
		System.out.println("Could not start process to read attached serial port.."+e);
			e.printStackTrace();
			return;
		}
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
		    try {
				awaitStart.await();
			} catch (InterruptedException e) {}
			TopicListInterface tli = asynchDemuxer.getTopic(AsynchDemuxer.topicNames.ANALOGPIN.val());
			MachineBridge mb = tli.getMachineBridge();
			for(int i = 0; i < 2 ; i++) {
			if( !mb.get().isEmpty() ) {
				int[] mr = (int[]) tli.getResult(mb.get().peekFirst());
				// wait for our pins to be displayed as we are potentially demuxxing numerous analogpin messages
				// look for Y input (throttle)
				if( mr[0] == joystickPinY) {// linear pin
					mb.get().takeFirst();
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
				if( mr[0] == joystickPinX) {// angular pin
					mb.get().takeFirst();
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
			}
			/*
			if( !pubdata.isEmpty() ) {
				int[] pubc = pubdata.takeFirst();
				geometry_msgs.Vector3 val = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
				val.setX(pubc[0]);
				val.setY(0);
				val.setZ(0);
				if( joymsg == null )
					joymsg = twistpub.newMessage();
				joymsg.setLinear(val);

				if( DEBUG) System.out.println("Set twist linear "+val.getX());
				geometry_msgs.Vector3 vala = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
				vala.setZ(pubc[1]);
				vala.setX(0);
				vala.setY(0);
				if( joymsg == null )
					joymsg = twistpub.newMessage();
				joymsg.setAngular(vala);
				if( DEBUG) System.out.println("Set twist angular "+val.getZ());	
			}
			*/
			// If we have a full set of coords publish, if we only get a partial then one of them
			// we in the deadband but that should not preclude a publication of valid coord plus deadband center
			// thus leaving no dead stick
			if( twistmsg != null && twistmsg.getLinear() != null && twistmsg.getAngular() != null) {
				twistpub.publish(twistmsg);
				if( DEBUG )
					System.out.println("Published:"+twistmsg);
				twistmsg = null;
			} else {
				if( twistmsg != null && twistmsg.getLinear() != null) {
					geometry_msgs.Vector3 val = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
					val.setZ((joystickMax/2));
					twistmsg.setAngular(val);
					if( DEBUG )
						System.out.println("Published fillin angular:"+twistmsg);
					twistmsg = null;
				} /*else {
					if( joymsg != null && joymsg.getAngular() != null) {
						geometry_msgs.Vector3 val = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
						val.setX((joystickMax/2));
						joymsg.setLinear(val);
						if( DEBUG )
							System.out.println("Published fillin linear:"+joymsg);
						joymsg = null;
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

class fileReader implements Runnable {
	public volatile boolean shouldRun = true;
	
	@Override
	public void run() {
		while(shouldRun) {
		try {
			FileReader fis = new FileReader("/home/jg/coords");
			BufferedReader br = new BufferedReader(fis);
			String s = br.readLine();
			br.close();
			fis.close();
			System.out.println(s);
			
			String left = s.substring(0,s.indexOf(","));
			String right = s.substring(s.indexOf(",")+1);
			System.out.println(left+","+right);
			int l = Integer.parseInt(left,10);
			int r = Integer.parseInt(right,10);
			//pubdata.addLast(new int[]{l,r});
			Thread.sleep(5);
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		}
	}
	
}
}
