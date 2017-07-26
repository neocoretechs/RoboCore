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

import com.neocoretechs.robocore.machine.bridge.AnalogPinListener;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.serialreader.IMUSerialDataPort;


/**
 * Takes readings from the IMU DataPort and packages them for publication on the ROS bus
 * @author jg
 */
public class IMUPubs extends AbstractNodeMain  {
	private static final boolean DEBUG = true;
	float volts;
	Object statMutex = new Object(); 
	Object navMutex = new Object();
	private String host;
	private InetSocketAddress master;
	private CountDownLatch awaitStart = new CountDownLatch(1);
	geometry_msgs.Twist twistmsg = null;
	//public CircularBlockingDeque<int[]> pubdata = new CircularBlockingDeque<int[]>(16);
	//private static int[] data = new int[]{-10,10,100};

	final static int joystickMin = 0;
	final static int joystickMax = 1024;
	final static int motorSpeedMax = 1000;
	// scale it to motor speed
	final static int sf = motorSpeedMax / (joystickMax/2);
	static IMUSerialDataPort imuPort;
	
	public IMUPubs(String host, InetSocketAddress master) {
		this.host = host;
		this.master = master;
		NodeConfiguration nc = build();
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, nc);
	    try {
			awaitStart.await();
		} catch (InterruptedException e) {}
	    
	}
	
	public IMUPubs(String[] args) {
		CommandLineLoader cl = new CommandLineLoader(Arrays.asList(args));
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, cl.build());
	    try {
			awaitStart.await();
		} catch (InterruptedException e) {}
	}
	
	public IMUPubs() {}
	
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
	imuPort = IMUSerialDataPort.getInstance();

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
	//if( mode.equals("startup")) 
	//}
	// This CancellableLoop will be canceled automatically when the node shuts
	// down.
	connectedNode.executeCancellableLoop(new CancellableLoop() {
		private long sequenceNumber, time1;
		@Override
		protected void setup() {
			sequenceNumber = 0;
			time1 = System.currentTimeMillis();
		}

		@Override
		protected void loop() throws InterruptedException {
			/*
			if( !pubdata.isEmpty() ) {
				int[] pubc = pubdata.takeFirst();
				geometry_msgs.Vector3 val = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
				val.setX(pubc[0]);
				val.setY(0);
				val.setZ(0);
				if( twistmsg == null )
					twistmsg = twistpub.newMessage();
				twistmsg.setLinear(val);

				if( DEBUG) System.out.println("Set twist linear "+val.getX());
				geometry_msgs.Vector3 vala = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
				vala.setZ(pubc[1]);
				vala.setX(0);
				vala.setY(0);
				if( twistmsg == null )
					twistmsg = twistpub.newMessage();
				twistmsg.setAngular(vala);
				if( DEBUG) System.out.println("Set twist angular "+val.getZ());	
			}
			*/

			try {
				getIMU();
				++sequenceNumber;
				if( System.currentTimeMillis() - time1 >= 1000) {
					time1 = System.currentTimeMillis();
					System.out.println("Samples per second:"+sequenceNumber);
					sequenceNumber = 0;
				}
				
				if( DEBUG )
					System.out.println("reading ACCEL");
				int[] accels = imuPort.readAccel();
				if( DEBUG && accels != null )
					System.out.println("Accel:"+accels[0]+" "+accels[1]+" "+accels[2]);
				if( accels != null ) {
					System.out.printf("X,Y,Z axis Acceleration : %d %d %d \r\n", accels[0],accels[1],accels[2]);
					geometry_msgs.Vector3 val = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
					val.setX(accels[0]);
					val.setY(accels[1]);
					val.setZ(accels[2]);
					if( twistmsg == null )
						twistmsg = twistpub.newMessage();
					twistmsg.setLinear(val);
				} else
					System.out.println("ACCEL ERROR");
				
				if( DEBUG )
					System.out.println("reading GYRO");
				int[] gyros = imuPort.readGyro();
				if( DEBUG && gyros != null)
					System.out.println("Gyros:"+gyros[0]+" "+gyros[1]+" "+gyros[2]);
				if( gyros != null ) {
					System.out.printf("X,Y,Z axis Of gyro Rotation : %d %d %d\r\n", gyros[0],gyros[1],gyros[2]);
					geometry_msgs.Vector3 vala = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
					vala.setZ(gyros[2]);
					vala.setX(gyros[0]);
					vala.setY(gyros[1]);
					if( twistmsg == null )
						twistmsg = twistpub.newMessage();
					twistmsg.setAngular(vala);
				} else
					System.out.println("GYRO ERROR");

				twistpub.publish(twistmsg);
		
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
	
			Thread.sleep(10);
		}
			
	});

}


public void getIMU() throws IOException{

	if( DEBUG )
		System.out.println("reading MAG");
	int[] mags = imuPort.readMag();
	if( DEBUG && mags!= null )
		System.out.println("Mag:"+mags[0]+" "+mags[1]+" "+mags[2]);
	
	if( DEBUG )
		System.out.println("reading EULER");
	double[] eulers = imuPort.readEuler();
	if( DEBUG && eulers != null)
		System.out.println("Eulers:"+eulers[0]+" "+eulers[1]+" "+eulers[2]);
	
	if( DEBUG )
		System.out.println("reading QUATERNION");
	double[] quats = imuPort.readQuaternion();
	if( DEBUG && quats != null)
		System.out.println("Quats:"+quats[0]+" "+quats[1]+" "+quats[2]+" "+quats[3]);
	
	if( DEBUG )
		System.out.println("reading TEMPERATURE");
	int temp = imuPort.readTemperature();
	if( DEBUG && temp != Integer.MAX_VALUE)
		System.out.println("Temp:"+temp);
	
	
	// Output data to screen

	if( mags != null )
		System.out.printf("X,Y,Z axis Magnetic field : %d %d %d \r\n", mags[0], mags[1], mags[2]);
	else
		System.out.println("MAG ERROR");
	if( eulers != null )
		System.out.printf("yaw, roll, pitch degrees: %f %f %f \r\n", eulers[0], eulers[1], eulers[2]);
	else
		System.out.println("FUSION ERROR");
	if( quats != null )
		System.out.printf("X, Y, Z, W degrees: %f %f %f %f\r\n", quats[0], quats[1], quats[2], quats[3]);
	else
		System.out.println("FUSION ERROR");
	if( temp != Integer.MAX_VALUE)
		System.out.printf("Temperature: %d \r\n", temp);
	else
		System.out.println("TEMPERATURE ERROR");
	
	
}	
class fileReader implements Runnable {
	public boolean shouldRun = true;
	
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
