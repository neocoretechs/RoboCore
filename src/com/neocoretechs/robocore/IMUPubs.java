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
 * Publishes user constructed Twist messages on the cmd_vel topic. Takes the demuxxed values
 * from the attached device that collects 2 axis data from joystick etc and translates to X linear and Z
 * angular values
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
	//try {
	//	setIMU();
	//} catch (IOException e) {
		// TODO Auto-generated catch block
	//	e.printStackTrace();
	//}
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
		private int sequenceNumber;
		@Override
		protected void setup() {
			sequenceNumber = 0;
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
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
	
			Thread.sleep(10);
		}
			
	});

}

public void setIMU() throws IOException {
	// Select OPR_MODE register
	// Accelerometer, Magnetometer and Gyro enabled
	if( DEBUG )
		System.out.println("...setting OPR_MODE");
	imuPort.write(IMUSerialDataPort.BNO055_OPR_MODE_ADDR, new byte[]{(byte)0x07}, false);
	// Select PWR_MODE register
	// Normal mode
	if( DEBUG )
		System.out.println("setting PWR_MODE");
	imuPort.write(IMUSerialDataPort.BNO055_PWR_MODE_ADDR, new byte[]{(byte)0x00}, false);
	// Select PAGE_ID register
	// Shift to Page-1
	if( DEBUG )
		System.out.println("setting PAGE_ID");
	imuPort.write(IMUSerialDataPort.BNO055_PAGE_ID_ADDR, new byte[]{(byte)0x01}, false);
	// Select ACC_CONFIG register
	// Range = 4G, B/W = 62.5, Normal mode
	if( DEBUG )
		System.out.println("setting ACC_CONFIG");
	imuPort.write((byte)0x08, new byte[]{(byte)0x0C}, false);
	// Select MAG_CONFIG register
	// Data o/p rate = 10 Hz, Regular mode, normal mode
	if( DEBUG )
		System.out.println("setting MAG_CONFIG");
	imuPort.write((byte)0x09, new byte[]{(byte)0x0B}, false);
	// Select GYRO_CONFIG1 register
	// Range = 2000 dps, B/W  = 32 Hz
	if( DEBUG )
		System.out.println("setting GYRO_CONFIG1");
	imuPort.write((byte)0x0A, new byte[]{(byte)0x38}, false);
	// Select GYRO_CONFIG2 register
	// Normal mode
	if( DEBUG )
		System.out.println("setting GYRO_CONFIG2");
	imuPort.write((byte)0x0B, new byte[]{(byte)0x00}, false);
	// Select PAGE_ID register
	// Shift to Page-0
	if( DEBUG )
		System.out.println("setting PAGE_ID to 0...");
	imuPort.write(IMUSerialDataPort.BNO055_PAGE_ID_ADDR, new byte[]{(byte)0x00}, false);
	try {
	Thread.sleep(500);
	} catch(InterruptedException ie) {}
}

public void getIMU() throws IOException{
	// Read 6 bytes of data from address 0x08(08)
	// xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
	byte[] data;
	/*
	if( DEBUG )
		System.out.println("reading ACCEL");

	data = imuPort.read((byte)0x08, (byte)6);

	// Convert the data
	int xAccl = ((data[1] & (byte)0xFF) * 256 + (byte)(data[0] & (byte)0xFF)) ;
	if(xAccl > 32767) {
		xAccl -= 65536;
	}	

	int yAccl = ((data[3] & (byte)0xFF) * 256 + (byte)(data[2] & (byte)0xFF)) ;
	if(yAccl > 32767) {
		yAccl -= 65536;
	}

	int zAccl = ((data[5] & (byte)0xFF) * 256 + (byte)(data[4] & (byte)0xFF)) ;
	if(zAccl > 32767) {
		zAccl -= 65536;
	}
*/
	if( DEBUG )
		System.out.println("reading MAG");
	// Read 6 bytes of data from address 0x0E(14)
	// xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
	data = imuPort.read((byte)0x0E, (byte)6);

	// Convert the data
	int xMag = ((data[1] & (byte)0xFF) * 256 + (byte)(data[0] & (byte)0xFF));
	if(xMag > 32767) {
		xMag -= 65536;
	}	

	int yMag = ((data[3] & 0xFF) * 256 + (byte)(data[2] & (byte)0xFF)) ;
	if(yMag > 32767) {
		yMag -= 65536;
	}

	int zMag = ((data[5] & 0xFF) * 256 + (byte)(data[4] & (byte)0xFF)) ;
	if(zMag > 32767) {
		zMag -= 65536;
	}

	if( DEBUG )
		System.out.println("reading GYRO");
	// Read 6 bytes of data from address 0x14(20)
	// xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
	data = imuPort.read((byte)0x14, (byte)6);

	// Convert the data
	int xGyro = ((data[1] & (byte)0xFF) * 256 + (byte)(data[0] & (byte)0xFF)) ;
	if(xGyro > 32767) {
		xGyro -= 65536;
	}

	int yGyro = ((data[3] & (byte)0xFF) * 256 + (byte)(data[2] & (byte)0xFF)) ;
	if(yGyro > 32767) {
		yGyro -= 65536;
	}

	int zGyro = ((data[5] & (byte)0xFF) * 256 + (byte)(data[4] & (byte)0xFF)) ;
	if(zGyro > 32767) {
		zGyro -= 65536;
	}

	// Output data to screen
	System.out.printf("X-axis Of Rotation : %d \r\n", xGyro);
	System.out.printf("Y-axis Of Rotation : %d \r\n", yGyro);
	System.out.printf("Z-axis Of Rotation : %d \r\n", zGyro);
	//System.out.printf("Acceleration in X-Axis : %d \r\n", xAccl);
	//System.out.printf("Acceleration in Y-Axis : %d \r\n", yAccl);
	//System.out.printf("Acceleration in Z-Axis : %d \r\n", zAccl);
	System.out.printf("Magnetic field in X-Axis : %d \r\n", xMag);
	System.out.printf("Magnetic field in Y-Axis : %d \r\n", yMag);
	System.out.printf("Magnetic field in Z-Axis : %d \r\n", zMag);	

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
