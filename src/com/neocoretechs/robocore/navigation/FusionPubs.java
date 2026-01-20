package com.neocoretechs.robocore.navigation;

import java.io.IOException;
import java.time.Instant;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import java.util.concurrent.CountDownLatch;

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
	float volts;
	Object statMutex = new Object(); 
	Object navMutex = new Object();
	private String mode="";
	private CountDownLatch awaitStart = new CountDownLatch(1);
	sensor_msgs.MagneticField magmsg = null;

	ArrayList<String> statPub = new ArrayList<String>();

	private CircularBlockingDeque<diagnostic_msgs.DiagnosticStatus> statusQueue = new CircularBlockingDeque<diagnostic_msgs.DiagnosticStatus>(1024);
	//public CircularBlockingDeque<int[]> pubdata = new CircularBlockingDeque<int[]>(16);
	long time1, startTime;
	long time2;

	IMUSerialDataPort imuDataPort;
	
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
	boolean acc_changed = false;
	boolean gyro_changed = false;
	boolean mag_changed = false;
	boolean temp_changed = false;
	boolean imu_changed = false;
	boolean system_needs_calibrating = true; // if mode is calibration and its first time through
	static final String REMAP_MODE_CALIBRATE="calibrate"; // REMAP_MODE value for calibration
	boolean display_revision = true;
	String IMUPort = "/dev/ttyS1";
	//-------------------------------------
	// ultrasonic ranging
	static long count = 0;
	int firepulse;
	int result_pin;
	static int err;
	static GpioNative gpio = null;
	public static String gpioChip = "gpiochip0";
	public static String spin = "PIN_22";
	public static boolean chipOpen = false;
	private static final int PULSE_DELAY = 2000;        // #2 us pulse = 2,000 ns
	private static final int PULSE = 10000;        // #10 us pulse = 10,000 ns
	private static final int SPEEDOFSOUND = 34029; // Speed of sound = 34029 cm/s
	private static final double MAX_RANGE = 4000; // 400 cm max range
	private static final int REJECTION_START = 1000;
	private static final float MIN_MOTION_STRENGTH = 5.0f;
	//public static VoxHumana speaker = null;
	private int WINSIZE = 20;
	public CircularBlockingDeque<String> pubdata = new CircularBlockingDeque<String>(WINSIZE);
	public CircularBlockingDeque<Point3f> pointWindow = new CircularBlockingDeque<Point3f>(WINSIZE);
	static enum MODE { HCSR04, URM37};
	static MODE SENSOR_TYPE = MODE.URM37;
	static String URMPort = null; //auto select, otherwise set to this from command line __URMPORT:=/dev/ttyxxx
	private Runnable readThread, pubThread;
	static class EulerTime {
		sensor_msgs.Imu ImuMessage;
		double[] eulers = null;
		int[] accels = null;
		int[] gyros = null;
		int[] mags = null;
		int temp = -1;
		double[] quats = null;
		int[] last_accels = null;
		int[] last_gyros = null;
		int[] last_mags = null;
		int last_temp = 0;
		double[] last_imu = null;
	}
	EulerTime eulers = new EulerTime();

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

		final std_msgs.Header header = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);
		// check command line remappings for __mode:=calibrate
		Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
		if( remaps.containsKey(REMAP_DEBUG) )
			if(remaps.get(REMAP_DEBUG).equals("true")) {
				DEBUG = true;
				IMUSerialDataPort.DEBUG = true;
			}
		if( remaps.containsKey(REMAP_MODE) )
			mode = remaps.get(REMAP_MODE);
		if( remaps.containsKey(REMAP_SYSTEM_CAL) )
			SYSTEM_CAL = Integer.parseInt(remaps.get(REMAP_SYSTEM_CAL));// system calibration
		if( remaps.containsKey(REMAP_ACC_CAL) )
			ACC_CAL = Integer.parseInt(remaps.get(REMAP_ACC_CAL)); // accel calibration
		if( remaps.containsKey(REMAP_GYRO_CAL) )
			GYRO_CAL = Integer.parseInt(remaps.get(REMAP_GYRO_CAL)); // gyro calibration
		if( remaps.containsKey(REMAP_MAG_CAL) )
			MAG_CAL = Integer.parseInt(remaps.get(REMAP_MAG_CAL));// magnetometer calibration
		if( remaps.containsKey(REMAP_IMU_TOL) )
			IMU_TOL = Integer.parseInt(remaps.get(REMAP_IMU_TOL));// number of decimal places of position readout
		if( remaps.containsKey(REMAP_IMU_FREQ) )
			IMU_FREQ = Integer.parseInt(remaps.get(REMAP_IMU_FREQ));// number of milliseconds between maximum publication times (should data remain constant)
		if( remaps.containsKey(REMAP_IMU_PORT) )
			IMUPort = remaps.get(REMAP_IMU_PORT);// IMU UART port (USB or tty)
		if( remaps.containsKey(REMAP_URM_PORT) ) {
			URMPort = remaps.get(REMAP_URM_PORT);
			readThread = new UltraRead(); // ultrasonic UART port (USB or tty)
		} else {
			readThread = new UltraPing(); // GPIO (GPIO library maps to pins on GPIO header)
		}
		SynchronizedThreadManager.getInstance().init(new String[]{"SYSTEM"});
		SynchronizedThreadManager.getInstance().spin(readThread, "SYSTEM");
		pubThread = () -> {
			while(true) {
				std_msgs.String ps = rangepub.newMessage();
				try {
					ps.setData(pubdata.take());
				} catch (InterruptedException e) {
					return;
				}
				rangepub.publish(ps);
			}
		};
		SynchronizedThreadManager.getInstance().spin(pubThread, "SYSTEM");
		// tell the waiting constructors that we have registered publishers if we are intercepting the command line build process
		awaitStart.countDown();
		// This CancellableLoop will be canceled automatically when the node shuts
		// down.
		connectedNode.executeCancellableLoop(new CancellableLoop() {
			private int sequenceNumber, lastSequenceNumber;
			org.ros.message.Time time = null;
			@Override
			protected void setup() {
				sequenceNumber = 0;
				lastSequenceNumber = 0;
				time1 = System.currentTimeMillis();
				startTime = time1;
				time2 = System.currentTimeMillis();
				imuDataPort = IMUSerialDataPort.getInstance(IMUPort);
				imuDataPort.setSYSTEM_CAL(SYSTEM_CAL);
				imuDataPort.setACC_CAL(ACC_CAL);
				imuDataPort.setGYRO_CAL(GYRO_CAL);
				imuDataPort.setMAG_CAL(MAG_CAL);
				imuDataPort.setIMU_TOL(IMU_TOL);
			}

			@Override
			protected void loop() throws InterruptedException {
				try {
					awaitStart.await();
				} catch (InterruptedException e) {}
				// Publish status to message bus, then, begin calibration if necessary
				// with prompts and status to status bus
				statPub.clear();
				/*
					statPub.add(e.getMessage());
					StackTraceElement[] se = e.getStackTrace();
					for(StackTraceElement ste: se)
						statPub.add(ste.getClassName()+","+ste.getMethodName()+","+ste.getFileName()+","+ste.getLineNumber());
					System.out.println(e);
					e.printStackTrace();
					new PublishDiagnosticResponse(connectedNode, statpub, statusQueue, "IMU INITIALIZATION", 
							diagnostic_msgs.DiagnosticStatus.WARN, statPub);
					while(!statusQueue.isEmpty()) {
						statpub.publish(statusQueue.takeFirst());
						Thread.sleep(1);
					}
					Thread.sleep(1000);
				 */
				if( mode.equals(REMAP_MODE_CALIBRATE) && system_needs_calibrating) {
					try {
						system_needs_calibrating = false; // assume optimistic success
						String calibString = "<<BEGIN IMU CALIBRATION KATAS!>>";
						do {
							statPub.add(calibString);
							new PublishDiagnosticResponse(connectedNode, statuspub, statusQueue, "IMU CALIBRATION", 
									diagnostic_msgs.DiagnosticStatus.WARN, statPub);
							while(!statusQueue.isEmpty()) {
								statuspub.publish(statusQueue.takeFirst());
								++sequenceNumber;
								if( DEBUG )
									System.out.println("Sequence:"+sequenceNumber);
								Thread.sleep(1);
							}
							statPub.clear();
						} while( !(calibString = imuDataPort.calibrate(imuDataPort.getCalibrationStatus())).contains("CALIBRATION ACHIEVED"));
						statPub.add(calibString);
						try {
							Thread.sleep(1);
						} catch (InterruptedException e) {}
						// publish final result to status message bus
						statPub.add(imuDataPort.displaySystemStatus(imuDataPort.getSystemStatus()));
						new PublishDiagnosticResponse(connectedNode, statuspub, statusQueue, "IMU CALIBRATION", 
								diagnostic_msgs.DiagnosticStatus.WARN, statPub);
						while(!statusQueue.isEmpty()) {
							statuspub.publish(statusQueue.takeFirst());
							++sequenceNumber;
							if( DEBUG )
								System.out.println("Sequence:"+sequenceNumber);
							Thread.sleep(1);
						}
					} catch (IOException e) {
						if(DEBUG)
							System.out.println("Cannot achieve proper calibration of IMU due to "+e);
						statPub.clear();
						statPub.add("Cannot achieve proper calibration of IMU due to:");
						statPub.add(e.getMessage());
						new PublishDiagnosticResponse(connectedNode, statuspub, statusQueue, "IMU CALIBRATION ERROR", diagnostic_msgs.DiagnosticStatus.ERROR, statPub);
						statPub.clear();
						e.printStackTrace();
					}
				} // calibration

				//
				// Begin IMU message processing
				//
				try {
					if(display_revision) {
						display_revision = false;
						statPub.add(imuDataPort.displayRevision(imuDataPort.getRevision()));
					}
					getIMU();
					++sequenceNumber;
					if( SAMPLERATE && System.currentTimeMillis() - time1 >= 1000) {
						time1 = System.currentTimeMillis();
						statPub.add("IMU Samples per second:"+(sequenceNumber-lastSequenceNumber));
						lastSequenceNumber = sequenceNumber;
						byte[] stat = imuDataPort.getCalibrationStatus();
						// If overall system status falls below 1, attempt an on-the-fly recalibration
						if( stat == null || stat[0] <= 1 ) {
							if( (time1-startTime) > 60000 ) { // give it 60 seconds to come up from last recalib
								startTime = time1; // start time is when we recalibrated last
								imuDataPort.resetCalibration();
								statPub.add("** SYSTEM RESET AND RECALIBRATED");
								stat = imuDataPort.getCalibrationStatus();
							}
						}
						statPub.add(imuDataPort.formatCalibrationStatus(stat));
					}
					if( hasDataChanged() ) {
						//MotionController.updatePID((float)eulers[0],  0.0f);
						header.setSeq(sequenceNumber);
						time = org.ros.message.Time.fromMillis(System.currentTimeMillis());
						header.setStamp(time);
						header.setFrameId(time.toString());
						synchronized(eulers) {
							eulers.ImuMessage = connectedNode.getTopicMessageFactory().newFromType(sensor_msgs.Imu._TYPE);
							magmsg = connectedNode.getTopicMessageFactory().newFromType(MagneticField._TYPE);
							eulers.ImuMessage.setHeader(header);
							magmsg.setHeader(header);
							
							if( eulers.accels != null ) {
								geometry_msgs.Vector3 val = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
								val.setX(eulers.accels[0]);
								val.setY(eulers.accels[1]);
								val.setZ(eulers.accels[2]);
								eulers.ImuMessage.setLinearAcceleration(val);
							} else {
								statPub.add("ACCEL ERROR");
							}

							if( eulers.gyros != null ) {
								geometry_msgs.Vector3 vala = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
								vala.setZ(eulers.gyros[2]);
								vala.setX(eulers.gyros[0]);
								vala.setY(eulers.gyros[1]);
								eulers.ImuMessage.setAngularVelocity(vala);
							} else {
								statPub.add("GYRO ERROR");
							}

							if( eulers.quats != null ) {
								geometry_msgs.Quaternion valq = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE);
								valq.setX(eulers.quats[0]);
								valq.setY(eulers.quats[1]);
								valq.setZ(eulers.quats[2]);
								valq.setW(eulers.quats[3]);
								eulers.ImuMessage.setOrientation(valq);
							} else {
								statPub.add("QUAT ERROR");
							}

							if(eulers != null) {
								eulers.ImuMessage.setCompassHeadingDegrees((float) eulers.eulers[0]);
								eulers.ImuMessage.setRoll((float)eulers.eulers[1]);
								eulers.ImuMessage.setPitch((float)eulers.eulers[2]);
								eulers.ImuMessage.setTemperature(eulers.temp);
							} else {
								statPub.add("EULER ERR");
							}

							//if( accels != null && gyros != null && quats != null && eulers != null && imu_changed) {
							//	if( DEBUG )
							//		System.out.println("Publishing IMU:"+eulers.euler);
							//	queueResponse(eulers.euler);
							//	Thread.sleep(1);
							//}

							// Mag
							if(eulers.mags != null && mag_changed) {
								geometry_msgs.Vector3 valm = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
								valm.setZ(eulers.mags[2]);
								valm.setX(eulers.mags[0]);
								valm.setY(eulers.mags[1]);
								magmsg.setMagneticField(valm);
								//queueResponse(magmsg);
								//Thread.sleep(1);
								//if( DEBUG )
								//	System.out.println("Publishing MAG:"+magmsg);
							} //else {
							//if(mags == null)
							//	statPub.add("MAG ERROR");
							//}
						}
					} // hasDataChanged

				} catch (IOException e) {
					System.out.println("IMU publishing loop malfunction "+e.getMessage()+" at "+LocalDateTime.ofInstant(Instant.ofEpochMilli(System.currentTimeMillis()), ZoneId.systemDefault()));
					statPub.add("IMU publishing loop malfunction:");
					statPub.add(e.getMessage());
					e.printStackTrace();
					try {
						imuDataPort.resetCalibration();
					} catch (IOException e1) {
						e1.printStackTrace();
					}
				}
			}

		});

	}
	/**
	 * Retrieve the data from the IMU
	 * @throws IOException
	 */
	public void getIMU() throws IOException{
		synchronized(eulers) {
			if( DEBUG )
				System.out.println("reading ACCEL");
			eulers.accels = imuDataPort.readAccel();
			if( DEBUG && eulers.accels != null )
				System.out.println("Accel:"+eulers.accels[0]+" "+eulers.accels[1]+" "+eulers.accels[2]);
			if( DEBUG )
				System.out.println("reading GYRO");
			eulers.gyros = imuDataPort.readGyro();
			if( DEBUG && eulers.gyros != null)
				System.out.println("Gyros:"+eulers.gyros[0]+" "+eulers.gyros[1]+" "+eulers.gyros[2]);
			if( DEBUG )
				System.out.println("reading MAG");
			eulers.mags = imuDataPort.readMag();
			if( DEBUG && eulers.mags!= null )
				System.out.println("Mag:"+eulers.mags[0]+" "+eulers.mags[1]+" "+eulers.mags[2]);

			if( DEBUG )
				System.out.println("reading EULER");
			//eulers = new EulerTime();
			eulers.eulers = imuDataPort.readEuler();
			if( DEBUG && eulers != null)
				System.out.println("Eulers:"+eulers.eulers[0]+" "+eulers.eulers[1]+" "+eulers.eulers[2]);

			if( DEBUG )
				System.out.println("reading QUATERNION");
			eulers.quats = imuDataPort.readQuaternion();
			if( DEBUG && eulers.quats != null)
				System.out.println("Quats:"+eulers.quats[0]+" "+eulers.quats[1]+" "+eulers.quats[2]+" "+eulers.quats[3]);

			if( DEBUG )
				System.out.println("reading TEMPERATURE");
			eulers.temp = imuDataPort.readTemperature();
			if( DEBUG && eulers.temp != Integer.MAX_VALUE)
				System.out.println("Temp:"+eulers.temp);

			// Output data to screen
			if( DEBUG ) {
				if( eulers.mags != null )
					System.out.printf("X,Y,Z axis Magnetic field : %d %d %d \r\n", eulers.mags[0], eulers.mags[1], eulers.mags[2]);
				else
					System.out.println("MAG ERROR");
				if( eulers != null )
					System.out.printf("yaw, roll, pitch degrees: %f %f %f \r\n", eulers.eulers[0], eulers.eulers[1], eulers.eulers[2]);
				else
					System.out.println("FUSION ERROR");
				if( eulers.quats != null )
					System.out.printf("X, Y, Z, W degrees: %f %f %f %f\r\n", eulers.quats[0], eulers.quats[1], eulers.quats[2], eulers.quats[3]);
				else
					System.out.println("FUSION ERROR");
				if( eulers.temp != Integer.MAX_VALUE)
					System.out.printf("Temperature: %d \r\n", eulers.temp);
				else
					System.out.println("TEMPERATURE ERROR");
			}
		}
	}
	/**
	 * Attempt to limit publishing of real time data by only sending on deltas
	 * @return true if any data has changed, then individual flags discern which data has changed and is to be published
	 */
	private boolean hasDataChanged() {
		boolean dataChanged = false;
		acc_changed = false;
		gyro_changed = false;
		mag_changed = false;
		imu_changed = false;
		temp_changed = false;
		synchronized(eulers) {
			if(eulers.last_accels == null || eulers.last_gyros == null || eulers.last_mags == null) {
				eulers.last_accels = new int[3];
				eulers.last_gyros = new int[3];
				eulers.last_mags = new int[3];
				eulers.last_imu = new double[3];
			}
			if(eulers.accels == null || eulers.gyros == null || eulers.mags == null)
				return true;
			if(eulers.accels[0] != eulers.last_accels[0] || eulers.accels[1] != eulers.last_accels[1] || eulers.accels[2] != eulers.last_accels[2] ) {
				eulers.last_accels[0] = eulers.accels[0];
				eulers.last_accels[1] = eulers.accels[1];
				eulers.last_accels[2] = eulers.accels[2];
				dataChanged = true;
				acc_changed = true;
			}
			if( eulers.gyros[0] != eulers.last_gyros[0] || eulers.gyros[1] != eulers.last_gyros[1] || eulers.gyros[2] != eulers.last_gyros[2]) {
				eulers.last_gyros[0] = eulers.gyros[0];
				eulers.last_gyros[1] = eulers.gyros[1];
				eulers.last_gyros[2] = eulers.gyros[2];
				dataChanged = true;
				gyro_changed = true;
			}
			if( eulers.mags[0] != eulers.last_mags[0] || eulers.mags[1] != eulers.last_mags[1] || eulers.mags[2] != eulers.last_mags[2]) {
				eulers.last_mags[0] = eulers.mags[0];
				eulers.last_mags[1] = eulers.mags[1];
				eulers.last_mags[2] = eulers.mags[2];
				dataChanged = true;
				mag_changed = true;
			}
			if( eulers.eulers[0] != eulers.last_imu[0] || eulers.eulers[1] != eulers.last_imu[1] || eulers.eulers[2] != eulers.last_imu[2]) {
				eulers.last_imu[0] = eulers.eulers[0];
				eulers.last_imu[1] = eulers.eulers[1];
				eulers.last_imu[2] = eulers.eulers[2];
				dataChanged = true;
				imu_changed = true;
			}
			if( eulers.temp != eulers.last_temp ) {
				eulers.last_temp = eulers.temp;
				dataChanged = true;
				temp_changed = true;
			}
		}
		// heartbeat
		if((System.currentTimeMillis() - time2) > IMU_FREQ) {
			time2 = System.currentTimeMillis();
			dataChanged = true;
			acc_changed = true;
			gyro_changed = true;
			mag_changed = true;
			imu_changed = true;
			temp_changed = true;
		}
		return dataChanged;
	}

	/**
	 * 
	 * Trigger the Range Finder and return the result
	 * https://www.tutorialspoint.com/arduino/arduino_ultrasonic_sensor.htm
	 * @return
	 */
	public static double getRangeHCSR04(int rangefindertrigger, int rangefinderresult ) {
		double distance = -1;
		int rejection_start = 0;
		try {
			//drive low and wait 2 us
			gpio.lineSetValue(rangefindertrigger,0);
			Thread.sleep(0, PULSE_DELAY);// wait 2 us
			// fire the trigger pulse, 1 then 0 with 10 microsecond wait 
			gpio.lineSetValue(rangefindertrigger,1);
			Thread.sleep(0, PULSE);// wait 10 us
			gpio.lineSetValue(rangefindertrigger,0);
			long stop, start;
			//echo will go 0 to 1 and need to save time for that. equivalent to pulseIn(HIGH)
			//duration = pulseIn(echoPin, HIGH);
			//if value is HIGH, pulseIn() waits for the pin to go from LOW to HIGH, starts timing, 
			//then waits for the pin to go LOW and stops timing. 
			while (gpio.lineGetValue(rangefinderresult) == 0) {
				Thread.sleep(0, 1);
				rejection_start++;
				if(rejection_start == REJECTION_START) 
					return -1; //infinity
			}
			start = System.nanoTime();
			rejection_start = 0;
			while (gpio.lineGetValue(rangefinderresult) == 1 ) {
				Thread.sleep(0, 1);
				rejection_start++;
				if(rejection_start == REJECTION_START) 
					return -1; //infinity
			}
			stop = System.nanoTime();
			long delta = (stop - start);
			//distance = ((((delta)/1e3)/2) / 29.1);
			distance = delta/5882.35294118;
		} catch (InterruptedException e) {    
			e.printStackTrace();
			System.out.println("Exception triggering range finder:"+e);
		}	 
		return distance;
	}

	/**
	 * 
	 * Trigger the Range Finder and return the result
	 * https://media.digikey.com/pdf/Data%20Sheets/DFRobot%20PDFs/SEN0001_Web.pdf
	 * @return
	 */
	public static double getRangeURM37(int rangefindertrigger,  int rangefinderresult ) {
		double distance = -1;
		int rejection_start = 0;
		long stop, start;
		try {
			//drive low and wait 2 us
			gpio.lineSetValue(rangefindertrigger,0);
			start = System.nanoTime();
			gpio.lineSetValue(rangefindertrigger,1);
			//echo will go 1 to 0 and need to save time for that. equivalent to pulseIn(LOW)
			//duration = pulseIn(echoPin, LOW);
			while (gpio.lineGetValue(rangefinderresult) == 1) {
				Thread.sleep(0, 1);
				rejection_start++;
				if(rejection_start == REJECTION_START) 
					return -1; //infinity
			}
			stop = System.nanoTime();
			long delta = (stop - start);
			distance = delta/50000; // 50 us per cm
		} catch (InterruptedException e) {    
			e.printStackTrace();
			System.out.println("Exception triggering range finder:"+e);
		}	 
		return distance;
	}
	private void queueResponse(double distance) {
		if( DEBUG ) {
			System.out.println("RangeFinder result ="+distance);//(Math.abs(distance-previousDistance)) +" cm");
		}
		synchronized(eulers) {
			if(eulers.ImuMessage != null) {
				double x = Math.sin(eulers.ImuMessage.getCompassHeadingDegrees()*0.01745329)*distance;
				double y = Math.cos(eulers.ImuMessage.getCompassHeadingDegrees()*0.01745329)*distance;
				Point3f winPoint = new Point3f((float)x,(float)y,(float)System.currentTimeMillis());
				pointWindow.addLast(winPoint);
				if(pointWindow.length() == WINSIZE) {
					ComputeVariance c = new ComputeVariance();
					c.leastVariance(pointWindow);
					pointWindow.poll();
					StringBuilder sb = new StringBuilder();
					// --- MOST RECENT DISTANCE ---
					sb.append("nowdistance=");
					float latestDist = 0.0f;
					float latestTime = Float.MIN_VALUE;
					for (Point3f p : pointWindow) {
					    if (p.t() > latestTime) {
					        latestTime = p.t();
					        latestDist = (float)Math.sqrt(p.x()*p.x() + p.y()*p.y());
					    }
					}
					sb.append(String.format("%3.1f", latestDist));
					// --- MIN DISTANCE ---
					sb.append(",mindistance=");
					float minDist = Float.MAX_VALUE;
					for (Point3f p : pointWindow) {
					    float d = (float)Math.sqrt(p.x()*p.x() + p.y()*p.y());
					    if (d < minDist)
					        minDist = d;
					}
					sb.append(String.format("%3.1f", minDist));
					// --- MAX DISTANCE ---
					sb.append(",maxdistance=");
					float maxDist = Float.MIN_VALUE;
					for (Point3f p : pointWindow) {
					    float d = (float)Math.sqrt(p.x()*p.x() + p.y()*p.y());
					    if (d > maxDist)
					        maxDist = d;
					}
					sb.append(String.format("%3.1f", maxDist));
					// --- AVERAGE DISTANCE ---
					sb.append(",avedistance=");
					float sumDist = 0.0f;
					for (Point3f p : pointWindow) {
					    sumDist += Math.sqrt(p.x()*p.x() + p.y()*p.y());
					}
					float avgDist = sumDist / pointWindow.size();
					sb.append(String.format("%3.1f", avgDist));
					/*
					sb.append(",confidence=");
					sb.append(c.getConfidence());
					sb.append(",noise_strength=");
					sb.append(c.getVariance1());
					sb.append(",jitter_strength=");
					sb.append(c.getVariance2());
					sb.append(",motion_strength=");
					sb.append(c.getVariance3());
					sb.append(",noise_axis_x=");
					sb.append(c.getEigvec1().x);
					sb.append(",noise_axis_y=");
					sb.append(c.getEigvec1().y);
					sb.append(",noise_axis_z=");
					sb.append(c.getEigvec1().z);
					sb.append(",jitter_axis_x=");
					sb.append(c.getEigvec2().x);
					sb.append(",jitter_axis_y=");
					sb.append(c.getEigvec2().y);
					sb.append(",jitter_axis_z=");
					sb.append(c.getEigvec2().z);
					*/
					sb.append(",motion_direction_x=");
					sb.append(c.getEigvec3().x);
					sb.append(",motion_direction_y=");
					sb.append(c.getEigvec3().y);
					sb.append(",motion_direction_z=");
					sb.append(c.getEigvec3().z);
					// --- WORLD-FRAME MOTION COMPUTATION ---
					double yaw = eulers.ImuMessage.getCompassHeadingDegrees()*0.01745329;
					double cos = Math.cos(yaw);
					double sin = Math.sin(yaw);
					// robot-frame PCA direction (unit vector)
					double v_x = c.getEigvec3().x;
					double v_y = c.getEigvec3().y;
					// world-frame direction (unit vector)
					double vwx =  cos * v_x - sin * v_y;
					double vwy =  sin * v_x + cos * v_y;
					// absolute world-frame velocity (scaled by PCA speed)
					double speed = Math.sqrt(c.getVariance3());
					double vwx_abs = vwx * speed;
					double vwy_abs = vwy * speed;
					sb.append(",world_direction_x=");
					sb.append(String.format("%3.3f", vwx));
					sb.append(",world_direction_y=");
					sb.append(String.format("%3.3f", vwy));
					sb.append(",world_velocity_x=");
					sb.append(String.format("%3.3f", vwx_abs));
					sb.append(",world_velocity_y=");
					sb.append(String.format("%3.3f", vwy_abs));
					sb.append("\r\n");
					if(vwx_abs >= MIN_MOTION_STRENGTH || vwy_abs >= MIN_MOTION_STRENGTH)
						pubdata.addLast(sb.toString());
					if(DEBUG)
						System.out.println(">>> Queuing:"+sb.toString());
				}  
			} else {
				if(DEBUG)
					System.out.println("<<< euler is null..>>>");
			}
		}
	}
	public static void main(String[] args) {	 
		// Setup GPIO Pins 
		if(!chipOpen) {
			if((err = gpio.openChip(gpioChip)) < 0)
				throw new RuntimeException("chipOpen error "+err);	
			chipOpen = true;
		}
		//range finder pins 
		int rangefindertrigger = gpio.findChipLine("PIN_5");
		try {
			Thread.sleep(250);
		} catch (InterruptedException e) {}
		int rangefinderresult = 0;
		switch(SENSOR_TYPE) {
		case HCSR04:
			rangefinderresult = gpio.findChipLine("PIN_3");
			break;
		case URM37:
			rangefinderresult = gpio.findChipLine("PIN_3");
			break;
		default:
			break;
		}

		try {
			Thread.sleep(250);
		} catch (InterruptedException e) {}

		do {
			// Get the range
			double distance= -1;
			switch(SENSOR_TYPE) {
			case HCSR04:
				distance = getRangeHCSR04(rangefindertrigger, rangefinderresult);
				break;
			case URM37:
				distance = getRangeURM37(rangefindertrigger, rangefinderresult);
				break;
			default:
				break;
			}

			if( distance < MAX_RANGE) { // 400 cm, max range is 200 cm typical
				if( DEBUG ) {
					System.out.println("RangeFinder result ="+distance +" mm");
				}
			}
			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {}
		} while (true);

	}

	/**
	 * This class is used for the thread Runnable when using the ultrasonic distance sensor in purely GPIO mode.
	 * In this case a pin triggers the device to send an ultrasonic pulse, then another pin receives
	 * the return signal pulse and a TOF (time of flight) calculation is performed. This uses the GPIO library
	 * and so must run as root for aarch64 using the kernel GPIO library. It supports the HCSR04 or URM37 device
	 * controlled by SENSOR_TYPE class field. calls queueResponse with distance measure 
	 */
	class UltraPing implements Runnable {
		public volatile boolean shouldRun = true;
		@Override
		public void run() {
			if(gpio == null)
				gpio = new GpioNative();
			// Setup GPIO Pins 
			if(!chipOpen) {
				if((err = gpio.openChip(gpioChip)) < 0)
					throw new RuntimeException("chipOpen error "+err);	
				chipOpen = true;
			}
			// get handle to line struct
			int pin = gpio.findChipLine(spin);
			if(pin < 0)
				throw new RuntimeException("findChipLine error:"+pin);
			//range finder pins
			final int rangefindertrigger = gpio.lineRequestOutput(pin);
			try {
				Thread.sleep(250);
			} catch (InterruptedException e) {}
			int rangefinderresult = 0;;
			switch(SENSOR_TYPE) {
			case HCSR04:
				rangefinderresult = gpio.findChipLine("PIN_3");
				break;
			case URM37:
				rangefinderresult = gpio.findChipLine("PIN_3");
				break;
			default:
				break;
			}
			try {
				Thread.sleep(250);
			} catch (InterruptedException e) {}
			while(shouldRun) {
				try {
					// Get the range
					double distance= -1;
					switch(SENSOR_TYPE) {
					case HCSR04:
						distance = getRangeHCSR04(rangefindertrigger, rangefinderresult);
						break;
					case URM37:
						distance = getRangeURM37(rangefindertrigger, rangefinderresult);
						break;
					default:
						break;
					}
					//if( distance < MAX_RANGE) { 
					if( DEBUG ) {
						System.out.println("RangeFinder result ="+distance);//(Math.abs(distance-previousDistance)) +" cm");
					}
					queueResponse(distance);
					//}
					try {
						Thread.sleep(250);
					} catch (InterruptedException e) {}
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		}
	}
	/**
	 * This class is used with the URM37 in UART mode. It calls the {@link UltrasonicSerialDataPort} readDistance
	 * method and then queues the result queueResponse.
	 */
	class UltraRead implements Runnable {
		public volatile boolean shouldRun = true;	
		@Override
		public void run() {
			UltrasonicSerialDataPort usdp = null;
			if(URMPort == null)
				usdp = UltrasonicSerialDataPort.getInstance();
			else
				try {
					usdp = UltrasonicSerialDataPort.getInstance(URMPort);
				} catch (IOException e) {
					e.printStackTrace();
				}
			while(shouldRun) {
				try {
					// Get the range
					double distance = usdp.readDistance();
					//if( distance < MAX_RANGE) { // 500 cm for URM37
					queueResponse(distance);
					try {
						Thread.sleep(200);
					} catch (InterruptedException e) {}
				} catch (Exception e) {
					e.printStackTrace();
					try {
						Thread.sleep(200);
					} catch (InterruptedException e2) {}
				}
			}
		}
	}
}
