package com.neocoretechs.robocore.navigation;

import java.io.IOException;
import java.time.Duration;
import java.time.Instant;
import java.time.LocalDateTime;
import java.time.ZoneId;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Map;

import org.json.JSONObject;
import org.ros.concurrent.CancellableLoop;
import org.ros.internal.loader.CommandLineLoader;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import com.neocoretechs.robocore.GpioNative;
import com.neocoretechs.robocore.SynchronizedThreadManager;
import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;
import com.neocoretechs.robocore.marlinspike.PublishDiagnosticResponse;
import com.neocoretechs.robocore.pca.ComputeVariance;
import com.neocoretechs.robocore.pca.Point3f;
import com.neocoretechs.robocore.serialreader.IMUSerialDataPort;
import com.neocoretechs.robocore.serialreader.UltrasonicSerialDataPort;

import diagnostic_msgs.DiagnosticStatus;
import sensor_msgs.Imu;
import std_msgs.Header;

/**
 * Takes readings from the IMU DataPort and packages them for publication on a circular blocking queue.
 * IMUSerialDataPort class is oriented toward the Bosch BNO055 in UART mode. The URM37 can be either GPIO  or UART (see ctors).
 * Option also exists for calibration mode, wherein a calibration kata must be performed until values reach their desired
 * levels, as set by the parameters SYSTEM_CAL, ACC_CAL, GYRO_CAL, MAG_CAL, and calibration file is written to the file
 * defined in IMUSerialDataPort. MAKE SURE TO DELETE FILE BEFORE CALIBRATION, as it is otherwise locked for update. 
 * Deleting file will reset CALIBRATION VALUES TO ZERO.. After successful calibration, IMU enters normal data publishing loop mode.
 * <p>
 * Ultrasonic range finder:
 * <p>
 * Use the jSerialComm or libgpio GPIO libraries to drive an ultrasonic range finder attached to 
 * the UART pins or the GPIO pins 2 and 3 on linux. <p>
 * If GPIO then Pin2 is the trigger and pin 3 is the resulting signal. <br>
 * In this case, to use the gpio library instead of a USB port YOU MUST RUN AS ROOT.
 * The trigger pulse goes high to low with a 10 microsecond wait.
 * There is a 2 second window where the result pin goes from low to high that we use as our interval
 * for sensing an object.
 * We use a 250 millisecond delay between trigger and result pin provisioning with trigger set to low and result pulled up.
 * From there we enter the loop to acquire the ranging checking for a 300cm maximum distance.
 * If we get something that fits these parameters we publish to the queue. We can also shut down
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
public class FusionIMURange   {
	private static boolean DEBUG = false;
	private static boolean SHOWQUEUE = true;
	private static final boolean SAMPLERATE = true; // display pubs per second
	private static final double G = 9.80665;
	private static final double DEG2RAD = 0.017453292519943295;
	private String mode="";

	sensor_msgs.MagneticField magmsg = new sensor_msgs.MagneticField();

	long time1, startTime;
	long epoch = System.currentTimeMillis();
	Instant lastTime = Instant.now();

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
	double[] last_accels = new double[3];
	double[] last_gyros = new double[3];
	double[] last_mags = new double[3];
	double[] last_imu = new double[3];
	int last_temp = 0;
	public static boolean system_needs_calibrating = true; // if mode is calibration and its first time through
	static final String REMAP_MODE_CALIBRATE="calibrate"; // REMAP_MODE value for calibration
	boolean display_revision = true;
	String IMUPort = "/dev/ttyUSB0";
	String URMPort = "/dev/ttyS1"; 
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
	private static final double MIN_DIST_DELTA = 5; // cm min distance between point readings
	private static final int REJECTION_START = 1000;
	private static final double MIN_MOTION_STRENGTH = .2;
	private static final double MIN_ACCEL = .9;
	double last_distance = 0;
	private int WINSIZE = 20;
	private CircularBlockingDeque<EulerTime> eulerdata = new CircularBlockingDeque<EulerTime>(2);
	public CircularBlockingDeque<String> dataQueue = new CircularBlockingDeque<String>(WINSIZE);
	public CircularBlockingDeque<String> statusQueue = new CircularBlockingDeque<String>(WINSIZE);
	private CircularBlockingDeque<Point3f> pointWindow = new CircularBlockingDeque<Point3f>(WINSIZE);
	static enum MODE { HCSR04, URM37};
	static MODE SENSOR_TYPE = MODE.URM37;
	private Runnable readThread, pubThread, timerThread;
	private int STALE_READING_SECONDS = 60;
	public static double EULERS_EPS0 = 5.0;
	public static double EULERS_EPS1 = 2.5;
	public static double EULERS_EPS2 = 2.5;
	public static double GYROS_EPS0 = .3;
	public static double GYROS_EPS1 = .4;
	public static double GYROS_EPS2 = .2;
	public static double ACCELS_EPS0 = .5;
	public static double ACCELS_EPS1 = .5;
	public static double ACCELS_EPS2 = .3;
	
	static class EulerTime {
		sensor_msgs.Imu ImuMessage = new Imu();
		double[] eulers = new double[3];
		double[] accels = new double[3];
		double[] gyros = new double[3];
		double[] mags = new double[3];
		double[] quats = new double[4];
		int temp = -1;
		public EulerTime(Time time, int seq) {
			Header header = new Header();
			header.setStamp(time);
			header.setSeq(seq);
			ImuMessage.setHeader(header);
		}
	}

	public FusionIMURange(boolean uart) {	
		init(uart);
	}
	public FusionIMURange(String urmPort) {
		this.URMPort = urmPort;
		init(false);
	}
	public FusionIMURange(String imuPort, String urmPort) {
		this.IMUPort = imuPort;
		this.URMPort = urmPort;
		init(true);
	}
	private void init(boolean uart) {
		if(uart) {
			readThread = new UltraRead(); // ultrasonic UART port (USB or tty)
		} else {
			readThread = new UltraPing(); // GPIO (GPIO library maps to pins on GPIO header)
		}
		pubThread = new ultraPub();
		timerThread = new TimeRefresh();
		
		SynchronizedThreadManager.getInstance().init(new String[]{"IMU","URS","TIMER"});
		SynchronizedThreadManager.getInstance().spin(readThread, "URS");
		SynchronizedThreadManager.getInstance().spin(pubThread, "IMU");
		SynchronizedThreadManager.getInstance().spin(timerThread, "TIMER");
		// tell the waiting constructors that we have registered publishers if we are intercepting the command line build process
		while(!((Notifier)pubThread).isStarted())
			try {
				if(DEBUG)
					System.out.println("init waiting for IMU readThread..");
				Thread.sleep(1000);
			} catch (InterruptedException e) {break;}
		
		while(!((Notifier)readThread).isStarted())
			try {
				if(DEBUG)
					System.out.println("init waiting for Ultrasonic readThread..");
				Thread.sleep(1000);
			} catch (InterruptedException e) {break;}
		
		while(!((Notifier)timerThread).isStarted())
			try {
				if(DEBUG)
					System.out.println("init waiting for timeThread..");
				Thread.sleep(1000);
			} catch (InterruptedException e) {break;}
		
		if(DEBUG)
			System.out.println("init..");
	}
	/**
	 * @return the iMUPort
	 */
	public String getIMUPort() {
		return IMUPort;
	}
	/**
	 * @param iMUPort the iMUPort to set
	 */
	public void setIMUPort(String iMUPort) {
		IMUPort = iMUPort;
	}
	/**
	 * @return the uRMPort
	 */
	public String getURMPort() {
		return URMPort;
	}
	/**
	 * @param URMPort the URMPort to set
	 */
	public void setURMPort(String uRMPort) {
		URMPort = uRMPort;
	}
	
	/**
	 * Retrieve the data from the IMU
	 * @param eulers the EulerTime instance
	 * @return true if data has changed from last batch of processing
	 * @throws IOException
	 */
	public boolean getIMU(EulerTime eulers) throws IOException{
		if( DEBUG )
			System.out.println("reading ACCEL");
		synchronized(eulers.accels) {
			eulers.accels = imuDataPort.readAccel();
			if(DEBUG)
				System.out.println("Accel:"+eulers.accels[0]+" "+eulers.accels[1]+" "+eulers.accels[2]);
		}
		if( DEBUG )
			System.out.println("reading GYRO");
		synchronized(eulers.gyros) {
			eulers.gyros = imuDataPort.readGyro();
			if( DEBUG)
				System.out.println("Gyros:"+eulers.gyros[0]+" "+eulers.gyros[1]+" "+eulers.gyros[2]);
		}
		if( DEBUG )
			System.out.println("reading MAG");
		synchronized(eulers.mags) {
			eulers.mags = imuDataPort.readMag();
			if(DEBUG)
				System.out.println("Mag:"+eulers.mags[0]+" "+eulers.mags[1]+" "+eulers.mags[2]);
		}
		if( DEBUG )
			System.out.println("reading EULER");
		synchronized(eulers.eulers) {
			eulers.eulers = imuDataPort.readEuler();
			if(DEBUG)
				System.out.println("Eulers:"+eulers.eulers[0]+" "+eulers.eulers[1]+" "+eulers.eulers[2]);
		}
		if( DEBUG )
			System.out.println("reading QUATERNION");
		synchronized(eulers.quats) {
			eulers.quats = imuDataPort.readQuaternion();
			if(DEBUG)
				System.out.println("Quats:"+eulers.quats[0]+" "+eulers.quats[1]+" "+eulers.quats[2]+" "+eulers.quats[3]);
		}
		if(DEBUG)
			System.out.println("reading TEMPERATURE");
		eulers.temp = imuDataPort.readTemperature();
		if(DEBUG && eulers.temp != Integer.MAX_VALUE)
			System.out.println("Temp:"+eulers.temp);
		
		boolean dataChanged = hasDataChanged(eulers);
		
		if(dataChanged) {
			
			synchronized(eulers.eulers) {
				synchronized(eulers.ImuMessage) {
					eulers.ImuMessage.setCompassHeadingDegrees((float)eulers.eulers[0]);
					eulers.ImuMessage.setRoll((float)eulers.eulers[1]);
					eulers.ImuMessage.setPitch((float)eulers.eulers[2]);
					eulers.ImuMessage.setTemperature(eulers.temp);
				}
			}
			synchronized(eulers.accels) {
					geometry_msgs.Vector3 val = new geometry_msgs.Vector3();
					val.setX(eulers.accels[0]);
					val.setY(eulers.accels[1]);
					val.setZ(eulers.accels[2]);
					synchronized(eulers.ImuMessage) {
						eulers.ImuMessage.setLinearAcceleration(val);
					}
			}
			synchronized(eulers.gyros) {
					geometry_msgs.Vector3 vala = new geometry_msgs.Vector3();
					vala.setZ(eulers.gyros[2]);
					vala.setX(eulers.gyros[0]);
					vala.setY(eulers.gyros[1]);
					synchronized(eulers.ImuMessage) {
						eulers.ImuMessage.setAngularVelocity(vala);
					}
			}
			synchronized(eulers.quats) {
					geometry_msgs.Quaternion valq = new geometry_msgs.Quaternion();
					valq.setX(eulers.quats[0]);
					valq.setY(eulers.quats[1]);
					valq.setZ(eulers.quats[2]);
					valq.setW(eulers.quats[3]);
					synchronized(eulers.ImuMessage) {
						eulers.ImuMessage.setOrientation(valq);
					}
			}
			// Mag
			synchronized(eulers.mags) {
				if(mag_changed) {
					geometry_msgs.Vector3 valm = new geometry_msgs.Vector3();
					valm.setZ(eulers.mags[2]);
					valm.setX(eulers.mags[0]);
					valm.setY(eulers.mags[1]);
					magmsg.setMagneticField(valm);
					statusQueue.addLast(magmsg.toString());
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

		// Output data to screen
		if( DEBUG ) {
			synchronized(eulers.mags) {
				System.out.printf("X,Y,Z axis Magnetic field : %f %f %f %n", eulers.mags[0], eulers.mags[1], eulers.mags[2]);
			}
			synchronized(eulers.eulers) {
				System.out.printf("yaw, roll, pitch degrees: %f %f %f %n", eulers.eulers[0], eulers.eulers[1], eulers.eulers[2]);
			}
			synchronized(eulers.quats) {
				System.out.printf("X, Y, Z, W degrees: %f %f %f %f %n", eulers.quats[0], eulers.quats[1], eulers.quats[2], eulers.quats[3]);
			}
			if(eulers.temp != Integer.MAX_VALUE)
				System.out.printf("Temperature: %d %n", eulers.temp);
			else
				System.out.println("TEMPERATURE ERROR");
		}
		return dataChanged;
	}
	/**
	 * Attempt to limit publishing of real time data by only sending on deltas
	 * @return true if any data has changed, then individual flags discern which data has changed and is to be published
	 */
	private boolean hasDataChanged(EulerTime eulers) {
		boolean dataChanged = false;
		acc_changed = false;
		gyro_changed = false;
		mag_changed = false;
		imu_changed = false;
		temp_changed = false;
		boolean timed_out = false;

		synchronized(eulers.accels) {
			if(Math.abs(eulers.accels[0] - last_accels[0]) > ACCELS_EPS0 ||
					Math.abs(eulers.accels[1] - last_accels[1]) > ACCELS_EPS1 || 
					Math.abs(eulers.accels[2] - last_accels[2]) > ACCELS_EPS2 ) {
				if(DEBUG)
					System.out.println("Accels delta:"+Math.abs(eulers.accels[0] - last_accels[0])+" "+
						Math.abs(eulers.accels[1] - last_accels[1])+" "+
						Math.abs(eulers.accels[2] - last_accels[2]));
				last_accels[0] = eulers.accels[0];
				last_accels[1] = eulers.accels[1];
				last_accels[2] = eulers.accels[2];
				dataChanged = true;
				acc_changed = true;
			}
		}
		synchronized(eulers.gyros) {
			if( Math.abs(eulers.gyros[0] - last_gyros[0]) > GYROS_EPS0 || 
					Math.abs(eulers.gyros[1] - last_gyros[1])  > GYROS_EPS1 || 
					Math.abs(eulers.gyros[2] - last_gyros[2]) > GYROS_EPS2) {
				if(DEBUG)
					System.out.println("Gyros delta:"+Math.abs(eulers.gyros[0] - last_gyros[0]) +" "+
						Math.abs(eulers.gyros[1] - last_gyros[1]) +" "+
						Math.abs(eulers.gyros[2] - last_gyros[2]));
				last_gyros[0] = eulers.gyros[0];
				last_gyros[1] = eulers.gyros[1];
				last_gyros[2] = eulers.gyros[2];
				dataChanged = true;
				gyro_changed = true;
			}
		}
		synchronized(eulers.mags) {
			if( Math.abs(eulers.mags[0] - last_mags[0]) > EULERS_EPS0 || 
					Math.abs(eulers.mags[1] - last_mags[1]) > EULERS_EPS1 || 
					Math.abs(eulers.mags[2] - last_mags[2]) > EULERS_EPS2) {
				last_mags[0] = eulers.mags[0];
				last_mags[1] = eulers.mags[1];
				last_mags[2] = eulers.mags[2];
				//dataChanged = true;
				mag_changed = true;
			}
		}
		synchronized(eulers.eulers) {
			if( Math.abs(eulers.eulers[0] - last_imu[0]) > EULERS_EPS0 || 
					Math.abs(eulers.eulers[1] - last_imu[1]) > EULERS_EPS1 || 
					Math.abs(eulers.eulers[2] - last_imu[2]) > EULERS_EPS2) {
				if(DEBUG)
					System.out.println("Eulers delta:"+ Math.abs(eulers.eulers[0] - last_imu[0])+" "+
						Math.abs(eulers.eulers[1] - last_imu[1]) +" "+ 
						Math.abs(eulers.eulers[2] - last_imu[2]));
				last_imu[0] = eulers.eulers[0];
				last_imu[1] = eulers.eulers[1];
				last_imu[2] = eulers.eulers[2];
				dataChanged = true;
				imu_changed = true;
			}
		}
		if( eulers.temp != last_temp ) {
			last_temp = eulers.temp;
			//dataChanged = true;
			temp_changed = true;
		}
		// heartbeat
		double lastIMUTimeSec;
		synchronized (eulers.ImuMessage) {
		    Instant imuStamp = toInstant(eulers.ImuMessage.getHeader().getStamp());
		    lastIMUTimeSec = Duration.between(imuStamp, Instant.now()).toNanos() / 1_000_000_000.0;
		}
		if(lastIMUTimeSec > STALE_READING_SECONDS) {
			if(DEBUG)
				System.out.println("Data changed last IMU reading as its "+lastIMUTimeSec+" seconds old.");
			timed_out = true;
			dataChanged = true;
			acc_changed = true;
			gyro_changed = true;
			mag_changed = true;
			imu_changed = true;
			temp_changed = true;
		}
		if(dataChanged) {
			if(DEBUG)
				System.out.printf("%s timeout=%b acc=%b gyro=%b mag=%b imu=%b temp=%b%n",
					this.getClass().getName(),timed_out,acc_changed,gyro_changed,mag_changed,imu_changed,temp_changed);
		}
		return dataChanged;
	}
	
	public static Instant toInstant(org.ros.message.Time t) {
	    return Instant.ofEpochSecond(t.secs, t.nsecs);
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
	/**
	 * Combine the euler latest and recent distance into the sliding window. When we reach threshold
	 * push the result onto the pubdata queue as string.
	 * Uses the IMU accelerometer to estimate robot velocity,
	 * rotate it into world-frame, subtract it from PCA world-frame motion,
	 * and emit a canonical semantic vector.
	 *
	 * <p>Pipeline stages:</p>
	 * <ol>
	 *   <li>Gravity compensation</li>
	 *   <li>Velocity integration</li>
	 *   <li>World-frame rotation</li>
	 *   <li>Ego-motion subtraction + semantic vector output</li>
	 * </ol>
	 *
	 * <p>The semantic vector contains:</p>
	 * <ul>
	 *   <li>Robot-frame PCA direction</li>
	 *   <li>World-frame PCA direction</li>
	 *   <li>World-frame relative velocity</li>
	 *   <li>Robot world-frame velocity</li>
	 *   <li>Absolute object world-frame velocity</li>
	 *   <li>Distance metrics</li>
	 *   <li>Jitter / noise / confidence</li>
	 * </ul>
	 *
	 * <p>The primary purpose is to compensate for robot motion in the range-finding pipeline. As a result:</p>
	 * <ul>
	 *   <li>motion_strength stops spiking when the robot moves</li>
	 *   <li>world_vel_x / world_vel_y become physically correct</li>
	 *   <li>Stationary objects appear stationary</li>
	 *   <li>Moving objects track correctly even while the robot turns</li>
	 *   <li>An LLM can now reason about true world-frame motion</li>
	 * </ul>
	 *
	 * @param eulers The most recent IMU packet
	 * @param distance The most recent distance measure
	 */
	private void queueResponse(EulerTime eulers, double distance) {
		if(last_distance == 0)
			last_distance = distance;
		if(Math.abs(distance - last_distance) <= MIN_DIST_DELTA) {
			if( DEBUG ) {
				System.out.println("RangeFinder result ="+distance);//(Math.abs(distance-lastDistance)) +" cm");
			}
			return;
		}
		last_distance = distance;
		if( DEBUG ) {
			System.out.println("RangeFinder result ="+distance);//(Math.abs(distance-lastDistance)) +" cm");
		}
		
		double pitch, roll, compassHeadingDegrees, accelX, accelY, accelZ;
		synchronized(eulers.ImuMessage) {
			//int lastIMUTime = Time.fromMillis(System.currentTimeMillis()).subtract(eulers.ImuMessage.getHeader().getStamp()).secs;
			//if(lastIMUTime > STALE_READING_SECONDS) {
			//	if(DEBUG)
			//		System.out.println("Discarding last IMU reading as its "+lastIMUTime+" seconds old.");
			//	return;
			//}
			compassHeadingDegrees = eulers.ImuMessage.getCompassHeadingDegrees();
			pitch = eulers.ImuMessage.getPitch();
			roll = eulers.ImuMessage.getRoll();
			accelX = eulers.ImuMessage.getLinearAcceleration().getX();
			accelY = eulers.ImuMessage.getLinearAcceleration().getY();
			accelZ = eulers.ImuMessage.getLinearAcceleration().getZ();
		}
		double accel = 0;
		double axCorrected = 0;
		double ayCorrected = 0;
		double vwx_abs = 0, vwy_abs = 0;
		double x = Math.sin(compassHeadingDegrees*DEG2RAD)*distance;
		double y = Math.cos(compassHeadingDegrees*DEG2RAD)*distance;
		double delta = Duration.between(lastTime, Instant.now()).toNanos() / 1_000_000_000.0;
		JSONObject sb = new JSONObject();
		//StringBuilder sb = new StringBuilder();
		//
		Point3f winPoint = new Point3f((float)x,(float)y,(float)delta);
		pointWindow.addLast(winPoint);
		if(pointWindow.length() == WINSIZE) {
			//
			float latestDist = 0.0f;
			float latestTime = Float.MIN_VALUE;
			// --- MIN DISTANCE ---
			//sb.append(",mindistance=");
			float minDist = Float.MAX_VALUE;
			float maxDist = Float.MIN_VALUE;
			float sumDist = 0.0f;
			double minTime = Double.MAX_VALUE;
			double maxTime = Double.MIN_VALUE;
			double sumTime = 0.0f;
			for (Point3f p : pointWindow) {
				float d = (float)Math.sqrt(p.x()*p.x() + p.y()*p.y());
				if (p.t() > latestTime) {
					latestTime = p.t();
					latestDist = d;
				}
				if (d < minDist)
					minDist = d;
				if (d > maxDist)
					maxDist = d;
				if(p.t() < minTime)
					minTime = p.t();
				if(p.t() > maxTime)
					maxTime = p.t();
				sumDist += Math.sqrt(p.x()*p.x() + p.y()*p.y());
				sumTime += p.t();
			}
			sb.put("nowtime",String.format("%s",lastTime.plusNanos((long) latestTime)));
			sb.put("nowdistance",Float.parseFloat(String.format("%3.3f", latestDist)));
			sb.put("mindistance",Float.parseFloat(String.format("%3.3f", minDist)));
			sb.put("maxdistance",Float.parseFloat(String.format("%3.3f", maxDist)));
			sb.put("mintime",String.format("%s",lastTime.plusNanos((long) minTime)));
			sb.put("maxtime",String.format("%s",lastTime.plusNanos((long) maxTime)));
			// --- AVERAGE DISTANCE ---
			//sb.append(",avedistance=");
			float avgDist = sumDist / pointWindow.size();
			double avgTime = sumTime / pointWindow.size();
			sb.put("avedistance",Float.parseFloat(String.format("%3.3f", avgDist)));
			sb.put("avetime",String.format("%s", avgTime));
			CircularBlockingDeque<Point3f> newWindow = new CircularBlockingDeque<Point3f>(pointWindow.size());
			for(int i = 0; i < pointWindow.size(); i++) {
				Point3f px = pointWindow.get(i);
				double td = px.t() - minTime;
				Point3f nx = new Point3f(px.x(), px.y(), (float)td);
				newWindow.add(i, nx);
			}
			ComputeVariance c = new ComputeVariance();
			c.leastVariance(newWindow);
			pointWindow.poll();
			// robot-frame PCA direction (unit vector)
			double v_x = c.getEigvec3().x;
			double v_y = c.getEigvec3().y;
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
			//sb.append(",motion_direction_x=");
			sb.put("motion_direction_x",Float.parseFloat(String.format("%3.3f",c.getEigvec3().x)));
			//sb.append(",motion_direction_y=");
			sb.put("motion_direction_y",Float.parseFloat(String.format("%3.3f",c.getEigvec3().y)));
			//sb.append(",motion_direction_z=");
			sb.put("motion_direction_z",Float.parseFloat(String.format("%3.3f",c.getEigvec3().z)));
			//
			// --- WORLD-FRAME MOTION COMPUTATION ---
			// determine if any forward or lateral acceleration is taking place
			accel = Math.sqrt(accelX*accelX + accelY*accelY);
			if(accel > MIN_ACCEL) {
				// 1) Gravity compensation (horizontal plane)
				axCorrected = accelX - G * Math.sin(pitch*DEG2RAD);
				ayCorrected = accelY + G * Math.sin(roll*DEG2RAD);
				// 2) Integrate accel -> robot-frame velocity
				v_x += axCorrected * (maxTime-minTime);
				v_y += ayCorrected * (maxTime-minTime);
				// Simple drift damping
				v_x *= 0.98;
				v_y *= 0.98;
			}
			sb.put("robot_acceleration_x",Float.parseFloat(String.format("%3.3f", axCorrected)));
			//sb.append(",world_direction_y=");
			sb.put("robot_acceleration_y",Float.parseFloat(String.format("%3.3f", ayCorrected)));
			double yaw = compassHeadingDegrees*DEG2RAD;
			double cos = Math.cos(yaw);
			double sin = Math.sin(yaw);
			// world-frame direction (unit vector)
			double vwx =  cos * v_x - sin * v_y;
			double vwy =  sin * v_x + cos * v_y;
			// absolute world-frame velocity (scaled by PCA speed)
			double speed = Math.sqrt(c.getVariance3());
			vwx_abs = Math.abs(vwx * speed);
			vwy_abs = Math.abs(vwy * speed);
			//sb.append(",world_direction_x=");
			sb.put("world_direction_x",Float.parseFloat(String.format("%3.3f", vwx)));
			//sb.append(",world_direction_y=");
			sb.put("world_direction_y",Float.parseFloat(String.format("%3.3f", vwy)));
			//sb.append(",world_velocity_x=");
			sb.put("world_velocity_x",Float.parseFloat(String.format("%3.3f", vwx_abs)));
			//sb.append(",world_velocity_y=");
			sb.put("world_velocity_y",Float.parseFloat(String.format("%3.3f", vwy_abs)));
		}	
		//sb.append("\r\n");
		if(DEBUG)
			System.out.println("Window full - Accel:"+accel+" vwx_abs="+vwx_abs+" vwy_abs="+vwy_abs+" yaw="+imuDataPort.getYawRateDegPerSec()+sb.toString());
		if(vwx_abs > 1e-8 && vwy_abs > MIN_MOTION_STRENGTH) {//MIN_MOTION_STRENGTH || vwy_abs >= MIN_MOTION_STRENGTH) {
			dataQueue.addLast(sb.toString());
			if(DEBUG || SHOWQUEUE)
				System.out.println(LocalDateTime.ofInstant(Instant.ofEpochMilli(System.currentTimeMillis()), ZoneId.systemDefault()).toString()+">>> Queueing:"+sb.toString());
		}
	}

	public static void main(String[] args) {
		if(args.length < 1) {;
			System.out.println("Usage: java FusionIMURange [gpio | uart]");
			System.exit(1);
		}
		FusionIMURange fimur = null;
		switch(args[0]) {
		case "gpio" -> fimur = new FusionIMURange(false);
		case "uart" -> fimur = new FusionIMURange(true);
		default -> System.exit(1);
		}	
		try {
			while(true)
				System.out.println(fimur.dataQueue.take());
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		SynchronizedThreadManager.startSupervisorThread();
	}
	
	static interface Notifier {
		public boolean isStarted();
	}
	/**
	 * This class is used for the thread Runnable when using the ultrasonic distance sensor in purely GPIO mode.
	 * In this case a pin triggers the device to send an ultrasonic pulse, then another pin receives
	 * the return signal pulse and a TOF (time of flight) calculation is performed. This uses the GPIO library
	 * and so must run as root for aarch64 using the kernel GPIO library. It supports the HCSR04 or URM37 device
	 * controlled by SENSOR_TYPE class field. calls queueResponse with distance measure 
	 */
	class UltraPing implements Runnable, Notifier {
		public volatile boolean shouldRun = true;
		boolean isRunning = false;
		@Override
		public boolean isStarted() {
			return isRunning;
		}
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
			isRunning = true;
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
					EulerTime et = eulerdata.peekLast();
					if(et != null) {
						queueResponse(et, distance);
					}
					//}
					try {
						Thread.sleep(150);
					} catch (InterruptedException e) {}
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		}
	}
	/**
	 * This class is used with the URM37 in UART mode. It calls the {@link UltrasonicSerialDataPort} readDistance
	 * method and then queues the result queueResponse after taking the latest IMU and combining the result.
	 */
	class UltraRead implements Runnable, Notifier {
		public volatile boolean shouldRun = true;
		boolean isRunning = false;
		@Override
		public boolean isStarted() {
			return isRunning;
		}
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
			isRunning = true;
			while(shouldRun) {
				try {
					// Get the range
					if(DEBUG)
						System.out.println("Waiting for distance read..");
					double distance = usdp.readDistance();
					EulerTime et = eulerdata.peekLast();
					if(et != null) {
						queueResponse(et, distance);
					}
					try {
						Thread.sleep(100);
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
	/**
	 * Read the IMU and queue the results onto the eulerdata mini-queue
	 */
	class ultraPub implements Runnable, Notifier {
		public volatile boolean shouldRun = true;
		boolean isRunning = false;
		private int sequenceNumber, lastSequenceNumber;
		org.ros.message.Time time = null;
		@Override
		public boolean isStarted() {
			return isRunning;
		}
		@Override
		public void run() {
			sequenceNumber = 0;
			lastSequenceNumber = 0;
			time1 = System.currentTimeMillis();
			startTime = time1;
			imuDataPort = IMUSerialDataPort.getInstance(IMUPort);
			// check initial calibration
			try {
				imuDataPort.setSYSTEM_CAL(SYSTEM_CAL);
				imuDataPort.setACC_CAL(ACC_CAL);
				imuDataPort.setGYRO_CAL(GYRO_CAL);
				imuDataPort.setMAG_CAL(MAG_CAL);
				imuDataPort.setIMU_TOL(IMU_TOL);
				byte[] stat = imuDataPort.getCalibrationStatus();
				// If overall system status falls below 1, attempt an on-the-fly recalibration
				if(stat == null || stat[1] == 0 || stat[2] == 0 || stat[3] == 0) {
					if(DEBUG)
						System.out.println("Calibration status="+stat[1]+" "+stat[2]+" "+stat[3]);
					imuDataPort.resetCalibration();
					calibrate();
				}
			} catch(IOException ioe) {
				ioe.printStackTrace();
			}
			isRunning = imuDataPort.isConnected();
			if(DEBUG)
				System.out.println(this.getClass().getName()+" running="+isRunning);
			// Publish status to message bus, then, begin calibration if necessary
			// with prompts and status to status bus
			dataQueue.clear();
			while(shouldRun) {
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
				// Begin IMU message processing
				//
				try {
					if(display_revision) {
						display_revision = false;
						statusQueue.addLast(imuDataPort.displayRevision(imuDataPort.getRevision()));
					}
					time1 = System.currentTimeMillis();
					time = org.ros.message.Time.fromMillis(time1);
					lastSequenceNumber = sequenceNumber;
					++sequenceNumber;
					EulerTime eulers = new EulerTime(time, sequenceNumber);
					if(getIMU(eulers)) {
						eulerdata.addLast(eulers);
						if(System.currentTimeMillis() - time1 >= 1000) {
							if(SAMPLERATE) {
								statusQueue.addLast("IMU Samples per second:"+(sequenceNumber-lastSequenceNumber));
							}
							// If overall system status falls below 1, attempt an on-the-fly recalibration
							try {
								// If any individual element falls below total usability, attempt an on-the-fly recalibration
								if( (time1-startTime) > 60000 ) { // give it 60 seconds to come up from last recalib
									byte[] stat = imuDataPort.getCalibrationStatus();
									startTime = time1; // start time is when we recalibrated last
									if( stat == null || stat[1] == 0 || stat[2] == 0 || stat[3] == 0) {
										imuDataPort.resetCalibration();
										statusQueue.addLast("** SYSTEM RESET AND RECALIBRATED");
										stat = imuDataPort.getCalibrationStatus();
										statusQueue.addLast(imuDataPort.formatCalibrationStatus(stat));
									}
								}
							} catch(IOException ioe) { // calibration
								ioe.printStackTrace();
							}
						}
					} // data changed - true return from getIMU

				} catch (IOException e) {
					System.out.println("IMU publishing loop malfunction "+e.getMessage()+" at "+LocalDateTime.ofInstant(Instant.ofEpochMilli(System.currentTimeMillis()), ZoneId.systemDefault()));
					statusQueue.addLast("IMU publishing loop malfunction:");
					statusQueue.addLast(e.getMessage());
					e.printStackTrace();
				}
			}
		}

		private void calibrate() {
			if(DEBUG)
				System.out.println(this.getClass().getName()+" calbration...");
			try {
				String calibString = "<<BEGIN IMU CALIBRATION KATAS!>>";
				do {
					statusQueue.addLast(calibString);
					byte[] cStat = imuDataPort.getCalibrationStatus();
					calibString = imuDataPort.calibrate(cStat);
				} while(!calibString.contains("CALIBRATION ACHIEVED"));
				statusQueue.addLast(calibString);
				try {
					Thread.sleep(1);
				} catch (InterruptedException e) {}
				// publish final result to status message bus
				byte[] status = imuDataPort.getCalibrationStatus();
				statusQueue.addLast(imuDataPort.displaySystemStatus(imuDataPort.getSystemStatus()));
				FusionIMURange.system_needs_calibrating = status[1] == 0 || status[2] == 0 || status[3] == 0;
			} catch (IOException e) {
				if(DEBUG)
					System.out.println("Cannot achieve proper calibration of IMU due to "+e);
				statusQueue.addLast("Cannot achieve proper calibration of IMU due to:");
				statusQueue.addLast(e.getMessage());
				e.printStackTrace();
			}
		}
	}
	
	class TimeRefresh implements Runnable, Notifier {
		public volatile boolean shouldRun = true;
		boolean isRunning = false;
		@Override
		public boolean isStarted() {
			return isRunning;
		}
		@Override
		public void run() {
			isRunning = true;
			while(shouldRun) {
				try {
					Thread.sleep(86400000);
					lastTime = Instant.now();
				} catch (Exception e) {
					e.printStackTrace();
					try {
						Thread.sleep(200);
					} catch (InterruptedException e2) {}
				}
			}
		}
	}
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
	public static class FusionPubs extends AbstractNodeMain {
		private static boolean DEBUG = false;
		private static final boolean SAMPLERATE = false; // display pubs per second
		private String mode="";
		public static FusionIMURange fusionIMU;
		public static RawIMUPubs rawIMUPubs;
		sensor_msgs.MagneticField magmsg = null;

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
		protected CircularBlockingDeque<DiagnosticStatus> statusQueue = new CircularBlockingDeque<DiagnosticStatus>(2);

		static String URMPort = "/dev/ttyS1"; //auto select, otherwise set to this from command line __URMPORT:=/dev/ttyxxx
		static String IMUPort = "/dev/ttyUSB1";

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

			connectedNode.executeCancellableLoop(new CancellableLoop() {
				@Override
				protected void setup() {
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
					
					// start the primary data generator
					fusionIMU = new FusionIMURange(IMUPort, URMPort);
					// start the status publisher
					new IMUStatusPubs(connectedNode, fusionIMU);
					rawIMUPubs = new RawIMUPubs(connectedNode, fusionIMU);

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
				}

				@Override
				protected void loop() throws InterruptedException {
					if(DEBUG)
						System.out.println("<<< Enter publishing loop >>>");
					//
					// Begin IMU message processing
					//
					EulerTime et = fusionIMU.eulerdata.peekLast();
					if(et != null) {
						synchronized(et.ImuMessage) {
							rawIMUPubs.imus.addLast(et.ImuMessage);
						}
					}
					try {
						Thread.sleep(100);
					} catch (InterruptedException e) {}
					String data = fusionIMU.dataQueue.poll();
					if(data != null) {
						if(DEBUG)
							System.out.println("Publishing data queue:"+fusionIMU.dataQueue.length());
						std_msgs.String sPub = new std_msgs.String();
						sPub.setData(data);
						rangepub.publish(sPub);
						if(DEBUG)
							System.out.println("Published data queue:"+fusionIMU.dataQueue.length());
					} else {
						Thread.sleep(20);
					}
					if(DEBUG)
						if(fusionIMU.dataQueue.isEmpty())
							System.out.println("empty");
						else
							System.out.println("queue size="+fusionIMU.dataQueue.length());
				}
			});
		}
	}
	/**
	 * Publish status using the command line from the connected FusionPubs node after its constructed.
	 * Spin up a new ComandLineLoader and NodeMainExecutor
	 */
	public static class IMUStatusPubs extends AbstractNodeMain {
		private static boolean DEBUG = false;
		private static final boolean SAMPLERATE = false; // display pubs per second
		public static FusionIMURange fusionIMU;
		Map<String, String> remaps;
		CircularBlockingDeque<DiagnosticStatus> diags = new CircularBlockingDeque<DiagnosticStatus>(20);
		//-------------------------------------
		public IMUStatusPubs(ConnectedNode connectedNode, FusionIMURange fusionIMURange) {
			fusionIMU = fusionIMURange;
			NodeConfiguration nc = connectedNode.getNodeConfiguration();
			CommandLineLoader cl0 = nc.getCommandLineLoader();
			remaps = cl0.getSpecialRemappings();
			CommandLineLoader cl = new CommandLineLoader(cl0, this.getClass().getName());
			NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
			nodeMainExecutor.execute(this, cl.build());
		}

		public IMUStatusPubs() {}

		public GraphName getDefaultNodeName() {
			return GraphName.of("pubs_imustatus");
		}

		@Override
		public void onStart(final ConnectedNode connectedNode) {
			//final RosoutLogger log = (Log) connectedNode.getLog();
			final Publisher<diagnostic_msgs.DiagnosticStatus> statuspub =
					connectedNode.newPublisher("robocore/status", diagnostic_msgs.DiagnosticStatus._TYPE);

			connectedNode.executeCancellableLoop(new CancellableLoop() {
				@Override
				protected void setup() {
					// check special command line remappings 				
					if( remaps.containsKey(REMAP_DEBUG) )
						if(remaps.get(REMAP_DEBUG).equals("true")) {
							DEBUG = true;
						}
				}

				@Override
				protected void loop() throws InterruptedException {
					if(DEBUG)
						System.out.println("<<< Enter publishing loop >>>");
					// Publish status to message bus, then, begin calibration if necessary
					// with prompts and status to status bus
					ArrayList<String> stats = new ArrayList<String>();
					fusionIMU.statusQueue.drainTo(stats);
					if(!stats.isEmpty()) {
						if(DEBUG)
							System.out.println("Publishing response queue:"+stats.size()+" residual:"+fusionIMU.statusQueue.size());
						new PublishDiagnosticResponse(connectedNode, statuspub, diags, "robocore/status", 
								diagnostic_msgs.DiagnosticStatus.OK, (Collection<String>)stats);
						if(DEBUG)
							System.out.println("Published response queue:"+stats.size());
					} else {
						Thread.sleep(200);
					}
				}
			});
		}
	}
	/**
	 * Publish the raw IMU data to the MotionController or whomever else needs it
	 */
	public static class RawIMUPubs extends AbstractNodeMain {
		private static boolean DEBUG = false;
		private static final boolean SAMPLERATE = false; // display pubs per second
		public static FusionIMURange fusionIMU;
		Map<String, String> remaps;
		public CircularBlockingDeque<sensor_msgs.Imu> imus = new CircularBlockingDeque<sensor_msgs.Imu>(20);
		//-------------------------------------
		public RawIMUPubs(ConnectedNode connectedNode, FusionIMURange fusionIMURange) {
			fusionIMU = fusionIMURange;
			NodeConfiguration nc = connectedNode.getNodeConfiguration();
			CommandLineLoader cl0 = nc.getCommandLineLoader();
			remaps = cl0.getSpecialRemappings();
			CommandLineLoader cl = new CommandLineLoader(cl0, this.getClass().getName());
			NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
			nodeMainExecutor.execute(this, cl.build());
		}

		public RawIMUPubs() {}

		public GraphName getDefaultNodeName() {
			return GraphName.of("pubs_imuraw");
		}

		@Override
		public void onStart(final ConnectedNode connectedNode) {
			//final RosoutLogger log = (Log) connectedNode.getLog();
			final Publisher<sensor_msgs.Imu> pubsimu = connectedNode.newPublisher("/sensor_msgs/Imu", sensor_msgs.Imu._TYPE);

			connectedNode.executeCancellableLoop(new CancellableLoop() {
				@Override
				protected void setup() {
					// check special command line remappings 				
					if( remaps.containsKey(REMAP_DEBUG) )
						if(remaps.get(REMAP_DEBUG).equals("true")) {
							DEBUG = true;
						}
				}

				@Override
				protected void loop() throws InterruptedException {
					if(DEBUG)
						System.out.println("<<< Enter publishing loop >>>");
					// Publish IMU to message bus
					sensor_msgs.Imu imuMessage = imus.take();
					if(DEBUG)
						System.out.println("Publishing queue:"+imus.size());
					pubsimu.publish(imuMessage);
				}
			});
		}
	}
}
