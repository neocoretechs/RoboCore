package com.neocoretechs.robocore.navigation;

import java.io.File;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Arrays;
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
import org.ros.internal.loader.CommandLineLoader;

import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;
import com.neocoretechs.robocore.marlinspike.PublishDiagnosticResponse;
import com.neocoretechs.robocore.serialreader.IMUSerialDataPort;

/**
 * Takes readings from the IMU DataPort and packages them for publication on the ROS bus.
 * Three messages will be published: sensor_msgs/Imu with the sensor_msgs.Imu class, sensor_msgs/Temperature with sensor_msgs.Temperature
 * and sensor_msgs/MagneticField with sensor_msgs.MagneticField.
 * IMUSerialDataPort class is oriented toward the Bosch BNO055.
 * Option also exists for calibration mode, wherein a calibration kata must be performed until values reach their desired
 * levels, as set by the parameters SYSTEM_CAL, ACC_CAL, GYRO_CAL, MAG_CAL, and calibration file is written to the file
 * defined in IMUSerialDataPort. MAKE SURE TO DELETE FILE BEFORE CALIBRATION, as it is otherwise locked for update. 
 * Deleting file will reset CALIBRATION VALUES TO ZERO.. After successful calibration, IMU enters normal data publishing loop mode.
 * 
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 */
public class IMUPubs extends AbstractNodeMain  {
	private static final boolean DEBUG = false;
	private static final boolean SAMPLERATE = true; // display pubs per second
	float volts;
	Object statMutex = new Object(); 
	Object navMutex = new Object();
	private String mode="";
	private String host;
	private InetSocketAddress master;
	private CountDownLatch awaitStart = new CountDownLatch(1);
	sensor_msgs.Imu imumsg = null;
	sensor_msgs.Temperature tempmsg = null;
	sensor_msgs.MagneticField magmsg = null;
	int[] accels = null;
	int[] gyros = null;
	int[] mags = null;
	int temp = -1;
	double[] eulers = null;
	double[] quats = null;
	//
	int[] last_accels = null;
	int[] last_gyros = null;
	int[] last_mags = null;
	int last_temp = 0;
	double[] last_imu = null;
	
	ArrayList<String> statPub = new ArrayList<String>();
	
	private CircularBlockingDeque<diagnostic_msgs.DiagnosticStatus> statusQueue = new CircularBlockingDeque<diagnostic_msgs.DiagnosticStatus>(1024);
	//public CircularBlockingDeque<int[]> pubdata = new CircularBlockingDeque<int[]>(16);
	long time1, startTime;
	long time2;

	IMUSerialDataPort imuPort;
	
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
	boolean display_revision = true;
	
	
	public IMUPubs(String[] args) {
		CommandLineLoader cl = new CommandLineLoader(Arrays.asList(args));
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, cl.build());
	}
	
	public IMUPubs() {}
	
	public GraphName getDefaultNodeName() {
		return GraphName.of("pubs_imu");
	}

@Override
public void onStart(final ConnectedNode connectedNode) {

	//final RosoutLogger log = (Log) connectedNode.getLog();
	
	final Publisher<sensor_msgs.Imu> imupub = connectedNode.newPublisher("/sensor_msgs/Imu", sensor_msgs.Imu._TYPE);
	final Publisher<sensor_msgs.MagneticField> magpub = connectedNode.newPublisher("/sensor_msgs/MagneticField", sensor_msgs.MagneticField._TYPE);
	final Publisher<sensor_msgs.Temperature> temppub = connectedNode.newPublisher("/sensor_msgs/Temperature", sensor_msgs.Temperature._TYPE);
	final std_msgs.Header header = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);
	// check command line remappings for __mode:=calibrate
	Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
	if( remaps.containsKey("__mode") )
		mode = remaps.get("__mode");
	if( remaps.containsKey("__system_cal") )
		SYSTEM_CAL = Integer.parseInt(remaps.get("__system_cal"));// system calibration
	if( remaps.containsKey("__acc_cal") )
		ACC_CAL = Integer.parseInt(remaps.get("__acc_del")); // accel calibration
	if( remaps.containsKey("__gyro_cal") )
		GYRO_CAL = Integer.parseInt(remaps.get("__gyro_cal")); // gyro calibration
	if( remaps.containsKey("__mag_cal") )
		MAG_CAL = Integer.parseInt(remaps.get("__mag_cal"));// magnetometer calibration
	if( remaps.containsKey("__imu_tol") )
		IMU_TOL = Integer.parseInt(remaps.get("__imu_tol"));// number of decimal places of position readout
	if( remaps.containsKey("__imu_freq") )
		IMU_FREQ = Integer.parseInt(remaps.get("__imu_freq"));// number of milliseconds between maximum publication times (should data remain constant)
	
	final Publisher<diagnostic_msgs.DiagnosticStatus> statpub =
			connectedNode.newPublisher("robocore/status", diagnostic_msgs.DiagnosticStatus._TYPE);
	
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
			imuPort = IMUSerialDataPort.getInstance();
			imuPort.setSYSTEM_CAL(SYSTEM_CAL);
			imuPort.setACC_CAL(ACC_CAL);
			imuPort.setGYRO_CAL(GYRO_CAL);
			imuPort.setMAG_CAL(MAG_CAL);
			imuPort.setIMU_TOL(IMU_TOL);
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
			if( mode.equals("calibrate") && system_needs_calibrating) {
				try {
					system_needs_calibrating = false; // assume optimistic success
					String calibString = "<<BEGIN IMU CALIBRATION KATAS!>>";
					do {
						statPub.add(calibString);
						new PublishDiagnosticResponse(connectedNode, statpub, statusQueue, "IMU CALIBRATION", 
								diagnostic_msgs.DiagnosticStatus.WARN, statPub);
						while(!statusQueue.isEmpty()) {
							statpub.publish(statusQueue.takeFirst());
							++sequenceNumber;
							if( DEBUG )
								System.out.println("Sequence:"+sequenceNumber);
							Thread.sleep(1);
						}
						statPub.clear();
					} while( !(calibString = imuPort.calibrate(imuPort.getCalibrationStatus())).contains("CALIBRATION ACHIEVED"));
					statPub.add(calibString);
		    		try {
						Thread.sleep(1);
					} catch (InterruptedException e) {}
		    		// publish final result to status message bus
		    		statPub.add(imuPort.displaySystemStatus(imuPort.getSystemStatus()));
		    		new PublishDiagnosticResponse(connectedNode, statpub, statusQueue, "IMU CALIBRATION", 
						diagnostic_msgs.DiagnosticStatus.WARN, statPub);
		    		while(!statusQueue.isEmpty()) {
		    			statpub.publish(statusQueue.takeFirst());
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
					new PublishDiagnosticResponse(connectedNode, statpub, statusQueue, "IMU CALIBRATION ERROR", 
						diagnostic_msgs.DiagnosticStatus.ERROR, statPub);
					statPub.clear();
					e.printStackTrace();
				}
			}
			try {
				if(display_revision) {
					display_revision = false;
					statPub.add(imuPort.displayRevision(imuPort.getRevision()));
				}
				getIMU();
				++sequenceNumber;
				if( SAMPLERATE && System.currentTimeMillis() - time1 >= 1000) {
					time1 = System.currentTimeMillis();
					statPub.add("IMU Samples per second:"+(sequenceNumber-lastSequenceNumber));
					lastSequenceNumber = sequenceNumber;
					byte[] stat = imuPort.getCalibrationStatus();
					// If overall system status falls below 1, attempt an on-the-fly recalibration
					if( stat == null || stat[0] <= 1 ) {
						if( (time1-startTime) > 60000 ) { // give it 60 seconds to come up from last recalib
							startTime = time1; // start time is when we recalibrated last
							imuPort.resetCalibration();
							statPub.add("** SYSTEM RESET AND RECALIBRATED");
							stat = imuPort.getCalibrationStatus();
						}
					}
					statPub.add(imuPort.formatCalibrationStatus(stat));
				}
				if( hasDataChanged() ) {
					//MotionController.updatePID((float)eulers[0],  0.0f);
					header.setSeq(sequenceNumber);
					time = org.ros.message.Time.fromMillis(System.currentTimeMillis());
					header.setStamp(time);
					header.setFrameId(time.toString());
					imumsg = imupub.newMessage();
					tempmsg = temppub.newMessage();
					magmsg = magpub.newMessage();
					imumsg.setHeader(header);
					tempmsg.setHeader(header);
					magmsg.setHeader(header);
				
					if( accels != null ) {
						geometry_msgs.Vector3 val = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
						val.setX(accels[0]);
						val.setY(accels[1]);
						val.setZ(accels[2]);
						imumsg.setLinearAcceleration(val);
					} else {
						statPub.add("ACCEL ERROR");
					}
	
					if( gyros != null ) {
						geometry_msgs.Vector3 vala = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
						vala.setZ(gyros[2]);
						vala.setX(gyros[0]);
						vala.setY(gyros[1]);
						imumsg.setAngularVelocity(vala);
					} else {
						statPub.add("GYRO ERROR");
					}

					if( quats != null ) {
						geometry_msgs.Quaternion valq = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE);
						valq.setX(quats[0]);
						valq.setY(quats[1]);
						valq.setZ(quats[2]);
						valq.setW(quats[3]);
						imumsg.setOrientation(valq);
						imumsg.setOrientationCovariance(eulers); // we send the euler angles through the covariance matrix
					} else {
						statPub.add("QUAT ERROR");
					}
					if( accels != null && gyros != null && quats != null && imu_changed) {
						if( DEBUG )
							System.out.println("Publishing IMU:"+imumsg);
						imupub.publish(imumsg);
						Thread.sleep(1);
					}
				
					// Mag
					if(mags != null && mag_changed) {
						geometry_msgs.Vector3 valm = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Vector3._TYPE);
						valm.setZ(mags[2]);
						valm.setX(mags[0]);
						valm.setY(mags[1]);
						magmsg.setMagneticField(valm);
						magpub.publish(magmsg);
						Thread.sleep(1);
						if( DEBUG )
							System.out.println("Publishing MAG:"+magmsg);
					} else {
						if(mags == null)
							statPub.add("MAG ERROR");
					}
					// Temp
					if(temp_changed) {
						tempmsg.setTemperature(temp);
						temppub.publish(tempmsg);
						if( DEBUG )
							System.out.println("Publishing TEMP:"+tempmsg);
					}
				
				} // hasDataChanged
				
			} catch (IOException e) {
				System.out.println("IMU publishing loop malfuntion "+e.getMessage());
				statPub.add("IMU publishing loop malfuntion:");
				statPub.add(e.getMessage());
				e.printStackTrace();
			}
			if(!statPub.isEmpty()) {
				new PublishDiagnosticResponse(connectedNode, statpub, statusQueue, "IMU STATUS", 
						diagnostic_msgs.DiagnosticStatus.WARN, statPub);
			}
			while(!statusQueue.isEmpty()) {
				statpub.publish(statusQueue.takeFirst());
				++sequenceNumber;
				if( DEBUG )
					System.out.println("Sequence:"+sequenceNumber);
				Thread.sleep(1);
			}
			Thread.sleep(1);
		}
			
	});

}
/**
 * Retrieve the data from the IMU
 * @throws IOException
 */
public void getIMU() throws IOException{
	if( DEBUG )
		System.out.println("reading ACCEL");
	accels = imuPort.readAccel();
	if( DEBUG && accels != null )
		System.out.println("Accel:"+accels[0]+" "+accels[1]+" "+accels[2]);
	if( DEBUG )
		System.out.println("reading GYRO");
	gyros = imuPort.readGyro();
	if( DEBUG && gyros != null)
		System.out.println("Gyros:"+gyros[0]+" "+gyros[1]+" "+gyros[2]);
	if( DEBUG )
		System.out.println("reading MAG");
	mags = imuPort.readMag();
	if( DEBUG && mags!= null )
		System.out.println("Mag:"+mags[0]+" "+mags[1]+" "+mags[2]);
	
	if( DEBUG )
		System.out.println("reading EULER");
	eulers = imuPort.readEuler();
	if( DEBUG && eulers != null)
		System.out.println("Eulers:"+eulers[0]+" "+eulers[1]+" "+eulers[2]);
	
	if( DEBUG )
		System.out.println("reading QUATERNION");
	quats = imuPort.readQuaternion();
	if( DEBUG && quats != null)
		System.out.println("Quats:"+quats[0]+" "+quats[1]+" "+quats[2]+" "+quats[3]);
	
	if( DEBUG )
		System.out.println("reading TEMPERATURE");
	temp = imuPort.readTemperature();
	if( DEBUG && temp != Integer.MAX_VALUE)
		System.out.println("Temp:"+temp);
	
	// Output data to screen
	if( DEBUG ) {
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
}
/**
 * Attemtp to limit publishing of real time data by only sending on deltas
 * @return true if any data has changed, then individual flags discern which data has changed and is to be published
 */
private boolean hasDataChanged() {
	boolean dataChanged = false;
	acc_changed = false;
	gyro_changed = false;
	mag_changed = false;
	imu_changed = false;
	temp_changed = false;
	if(last_accels == null || last_gyros == null || last_mags == null) {
		last_accels = new int[3];
		last_gyros = new int[3];
		last_mags = new int[3];
		last_imu = new double[3];
	}
	if(accels == null || gyros == null || mags == null)
		return true;
	if(accels[0] != last_accels[0] || accels[1] != last_accels[1] || accels[2] != last_accels[2] ) {
		last_accels[0] = accels[0];
		last_accels[1] = accels[1];
		last_accels[2] = accels[2];
		dataChanged = true;
		acc_changed = true;
	}
	if( gyros[0] != last_gyros[0] || gyros[1] != last_gyros[1] || gyros[2] != last_gyros[2]) {
		last_gyros[0] = gyros[0];
		last_gyros[1] = gyros[1];
		last_gyros[2] = gyros[2];
		dataChanged = true;
		gyro_changed = true;
	}
	if( mags[0] != last_mags[0] || mags[1] != last_mags[1] || mags[2] != last_mags[2]) {
		last_mags[0] = mags[0];
		last_mags[1] = mags[1];
		last_mags[2] = mags[2];
		dataChanged = true;
		mag_changed = true;
	}
	if( eulers[0] != last_imu[0] || eulers[1] != last_imu[1] || eulers[2] != last_imu[2]) {
		last_imu[0] = eulers[0];
		last_imu[1] = eulers[1];
		last_imu[2] = eulers[2];
		dataChanged = true;
		imu_changed = true;
	}
	if( temp != last_temp ) {
		last_temp = temp;
		dataChanged = true;
		temp_changed = true;
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

}
