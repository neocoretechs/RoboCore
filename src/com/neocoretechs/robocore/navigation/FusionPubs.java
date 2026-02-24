package com.neocoretechs.robocore.navigation;

import java.io.IOException;
import java.time.Instant;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import java.util.concurrent.CountDownLatch;

import org.json.JSONObject;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.ros.internal.loader.CommandLineLoader;

import com.neocoretechs.robocore.GpioNative;
import com.neocoretechs.robocore.SynchronizedThreadManager;
import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;
import com.neocoretechs.robocore.marlinspike.PublishDiagnosticResponse;
import com.neocoretechs.robocore.pca.ComputeVariance;
import com.neocoretechs.robocore.pca.Point3f;
import com.neocoretechs.robocore.serialreader.IMUSerialDataPort;
import com.neocoretechs.robocore.serialreader.UltrasonicSerialDataPort;

import sensor_msgs.MagneticField;

/**
 * Takes readings from the IMU DataPort, specified by the __IMUPORT:= command line param, and packages them for publication on the ROS bus.
 * Three messages will be published: sensor_msgs/range with the sensor_msgs.Imu class, sensor_msgs.Temperature
 * and sensor_msgs.MagneticField messages.
 * IMUSerialDataPort class is oriented toward the Bosch BNO055.
 * Option also exists for calibration mode, wherein a calibration kata must be performed until values reach their desired
 * levels, as set by the parameters SYSTEM_CAL, ACC_CAL, GYRO_CAL, MAG_CAL, and calibration file is written to the file
 * defined in IMUSerialDataPort. MAKE SURE TO DELETE FILE BEFORE CALIBRATION, as it is otherwise locked for update. 
 * Deleting file will reset CALIBRATION VALUES TO ZERO.. After successful calibration, IMU enters normal data publishing loop mode.
 * <p>
 * Ultrasonic range finder:
 * <p>
 * Use the jSerialComm or libgpio GPIO libraries to drive an ultrasonic range finder attached to 
 * the UART pins or the GPIO pins 2 and 3 on linux. <p>
 * The mode is controlled by the presence and value of command line parameter __URMPORT:= <p>
 * IF __URMPORT:= isnt specified, then Pin2 being the trigger and pin 3 being the resulting signal. <br>
 * In this case, to use the gpio library instead of a USB port YOU MUST RUN AS ROOT.
 * The trigger pulse goes high to low with a 10 microsecond wait.
 * There is a 2 second window where the result pin goes from low to high that we use as our interval
 * for sensing an object.
 * We use a 250 millisecond delay between trigger and result pin provisioning with trigger set to low and result pulled up.
 * From there we enter the loop to acquire the ranging checking for a 300cm maximum distance.
 * If we get something that fits these parameters we publish to the robocore/status bus. We can also shut down
 * publishing to the bus while remaining in the detection loop by sending a message to the alarm/shutdown topic.
 * Alternately, the URM37 uses a serial data protocol at 9600,8,N,1 {@link UltrasonicSerialDataPort}<p>
 * We are going to fuse with IMU yaw (compass heading) data to construct a sliding window upon which we will 
 * do PCA to produce condensed motion vectors.<p>
 * Starting with: <br>
 * x = sin(yaw*0.01745329)*dist <br>
 * y = cos(yaw*0.01745329)*dist <br>
 * We will form a sliding {@link Point3f} window of fused points x,y,time <br>
 * d_now = distance to closest object now<br>
 * d_avg = distance avg across time window<br>
 * d_min = distance min closest observed distance<br>
 * d_max = distance max farthest observed distance<br>
 * that add semantic content to the individual measurements. the result of PCA will be<p>
 * variance3 = dominant motion axis - motion strength <br>
 * variance2 = lateral jitter       <br>
 * variance1 = noise floor          <br>
 * <br>
 * eigenVector3 = principal motion direction - full 3D motion direction <br>
 * eigenVector2 = lateral jitter axis <br>
 * eigenvector1 = noise axis <br>
 * confidence = variance3 / (variance2 + variance1 + eps) - coherence of motion - high - coherent - low - noise<br>
 * scalar magnitude of motion along the principal axis = Math.sqrt(variance3) <br>
 * producing a data buffer of [d_now, d_avg, d_min, d_max, v_x, v_y, speed, jitter, noise, confidence]
 * v_x = The x component of the principal motion vector, derived from PCA on (x, y, t) Meaning:<br>
 * How much the object’s motion projects onto the robot’s forward axis.<br>
 * positive - object moving outward along the robot’s forward axis<br>
 * negative - object approaching<br>
 * magnitude - forward/backward motion strength<br>
 * This is not raw velocity, it’s the direction of motion in the robot’s local frame.<br>
 * v_y = The y component of the principal motion vector, again from PCA. Meaning:<br>
 * How much the object’s motion projects onto the robot’s lateral axis.<br>
 * positive - drifting to the robot’s left<br>
 * negative - drifting to the robot’s right<br>
 * magnitude - lateral motion strength<br>
 * This is incredibly useful for:<br>
 * tracking moving objects<br>
 * distinguishing straight in approach vs diagonal motion<br>
 * predicting future positions<br>
 * 
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021,2025
 */
public class FusionPubs extends AbstractNodeMain  {
	private static boolean DEBUG = false;
	private static final boolean SAMPLERATE = false; // display pubs per second
	private String mode="";
	public FusionIMURange fusionIMU;
	private CountDownLatch awaitStart = new CountDownLatch(1);
	sensor_msgs.MagneticField magmsg = null;

	ArrayList<String> statPub = new ArrayList<String>();

	long time1, startTime;
	long time2;

	static final String REMAP_URM_PORT = "__urmport";
	static final String REMAP_IMU_PORT = "__imuport";
	static final String REMAP_DEBUG = "__debug";
	static final String REMAP_MODE = "__mode";
	static final String REMAP_SYSTEM_CAL = "__system_cal";
	static final String REMAP_ACC_CAL = "__acc_cal";
	static final String REMAP_GYRO_CAL = "__gyro_cal";
	static final String REMAP_MAG_CAL = "__mag_cal";
	static final String REMAP_IMU_TOL = "__imu_tol";
	static final String REMAP_IMU_FREQ = "__imu_freq";
	int SYSTEM_CAL = 3; // system calibration
	int ACC_CAL = 3; // accel calibration
	int GYRO_CAL = 3; // gyro calibration
	int MAG_CAL = 3; // magnetometer calibration
	int IMU_TOL = 3; // number of decimal places of position readout
	int IMU_FREQ = 1000; // Update frequency, to publish regardless if data is unchanged

	boolean system_needs_calibrating = true; // if mode is calibration and its first time through
	static final String REMAP_MODE_CALIBRATE="calibrate"; // REMAP_MODE value for calibration
	boolean display_revision = true;
	//-------------------------------------

	//public static VoxHumana speaker = null;
	private int WINSIZE = 20;
	//public CircularBlockingDeque<String> pubdata = new CircularBlockingDeque<String>(WINSIZE);
	public CircularBlockingDeque<Point3f> pointWindow = new CircularBlockingDeque<Point3f>(WINSIZE);
	private CircularBlockingDeque<diagnostic_msgs.DiagnosticStatus> statusQueue = new CircularBlockingDeque<diagnostic_msgs.DiagnosticStatus>(1024);

	static String URMPort = "/dev/ttyS1"; //auto select, otherwise set to this from command line __URMPORT:=/dev/ttyxxx
	static String IMUPort = "/dev/ttyUSB0";
	//private Runnable readThread, pubThread;

	public FusionPubs(String[] args) {
		CommandLineLoader cl = new CommandLineLoader(Arrays.asList(args));
		NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
		nodeMainExecutor.execute(this, cl.build());
	}

	public FusionPubs() {}

	public GraphName getDefaultNodeName() {
		return GraphName.of("pubs_fusion");
	}

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		//final RosoutLogger log = (Log) connectedNode.getLog();
		final Publisher<std_msgs.String> rangepub  = connectedNode.newPublisher("/sensor_msgs/range", std_msgs.String._TYPE);
		final Publisher<diagnostic_msgs.DiagnosticStatus> statuspub =
				connectedNode.newPublisher("robocore/status", diagnostic_msgs.DiagnosticStatus._TYPE);

		// check command line remappings for __mode:=calibrate
		Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
		if( remaps.containsKey(REMAP_DEBUG) )
			if(remaps.get(REMAP_DEBUG).equals("true")) {
				DEBUG = true;
				IMUSerialDataPort.DEBUG = true;
			}
		if( remaps.containsKey(REMAP_IMU_PORT) )
			IMUPort = remaps.get(REMAP_IMU_PORT);// IMU UART port (USB or tty)
		if( remaps.containsKey(REMAP_URM_PORT) ) {
			URMPort = remaps.get(REMAP_URM_PORT);
		}

		fusionIMU = new FusionIMURange(IMUPort, URMPort);
		
		if( remaps.containsKey(REMAP_MODE) )
			mode = remaps.get(REMAP_MODE); // calibrate if this equals REMAP_MODE_CALIBRATE
		if( remaps.containsKey(REMAP_SYSTEM_CAL) )
			fusionIMU.SYSTEM_CAL = Integer.parseInt(remaps.get(REMAP_SYSTEM_CAL));// system calibration
		if( remaps.containsKey(REMAP_ACC_CAL) )
			fusionIMU.ACC_CAL = Integer.parseInt(remaps.get(REMAP_ACC_CAL)); // accel calibration
		if( remaps.containsKey(REMAP_GYRO_CAL) )
			fusionIMU.GYRO_CAL = Integer.parseInt(remaps.get(REMAP_GYRO_CAL)); // gyro calibration
		if( remaps.containsKey(REMAP_MAG_CAL) )
			fusionIMU.MAG_CAL = Integer.parseInt(remaps.get(REMAP_MAG_CAL));// magnetometer calibration
		if( remaps.containsKey(REMAP_IMU_TOL) )
			fusionIMU.IMU_TOL = Integer.parseInt(remaps.get(REMAP_IMU_TOL));// number of decimal places of position readout
		if( remaps.containsKey(REMAP_IMU_FREQ) )
			fusionIMU.IMU_FREQ = Integer.parseInt(remaps.get(REMAP_IMU_FREQ));// number of milliseconds between maximum publication times (should data remain constant)

		//SynchronizedThreadManager.getInstance().init(new String[]{"SYSTEM"});
		//pubThread = () -> {
		//	while(true) {
		//		std_msgs.String ps = rangepub.newMessage();
		//		try {
		//			ps.setData(pubdata.take());
		//		} catch (InterruptedException e) {
		//			return;
		//		}
		//		rangepub.publish(ps);
		//	}
		//};
		//SynchronizedThreadManager.getInstance().spin(pubThread, "SYSTEM");
		// tell the waiting constructors that we have registered publishers if we are intercepting the command line build process
		awaitStart.countDown();
		// This CancellableLoop will be canceled automatically when the node shuts
		// down.
		connectedNode.executeCancellableLoop(new CancellableLoop() {
			@Override
			protected void setup() {
			}

			@Override
			protected void loop() throws InterruptedException {
				try {
					awaitStart.await();
				} catch (InterruptedException e) {}
				// Publish status to message bus, then, begin calibration if necessary
				// with prompts and status to status bus
				statPub.clear();
				fusionIMU.statusQueue.drainTo(statPub);
				if(!statPub.isEmpty())
					new PublishDiagnosticResponse(connectedNode, statuspub, statusQueue, "robocore/status", 
							diagnostic_msgs.DiagnosticStatus.OK, statPub);
				//
				// Begin IMU message processing
				//
				fusionIMU.dataQueue.forEach(e-> {
					std_msgs.String sPub = new std_msgs.String();
					sPub.setData(e);
					rangepub.publish(sPub);
				});
			}
		});
	}

}
