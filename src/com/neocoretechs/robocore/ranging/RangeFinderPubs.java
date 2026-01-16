package com.neocoretechs.robocore.ranging;
import java.io.IOException;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.CountDownLatch;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import com.neocoretechs.robocore.serialreader.UltrasonicSerialDataPort;
import com.neocoretechs.robocore.GpioNative;
import com.neocoretechs.robocore.SynchronizedThreadManager;
import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;
import com.neocoretechs.robocore.pca.Point3f;
import com.neocoretechs.robocore.pca.ComputeVariance;

/**
 * Use the jSerialComm or libgpio GPIO libraries to drive an ultrasonic range finder attached to 
 * the UART pins or the GPIO pins 2 and 3 on linux. <p>
 * The mode is controlled by the presence and value of command line parameter __PORT:= <p>
 * IF PORT:= isnt specified, then Pin2 being the trigger and pin 3 being the resulting signal. <br>
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
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class RangeFinderPubs extends AbstractNodeMain {
	private static boolean DEBUG = true;
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
	static final String REMAP_DEBUG = "__debug";
	private CountDownLatch awaitStart = new CountDownLatch(1);
	//public static VoxHumana speaker = null;
	private int WINSIZE = 20;
	public ConcurrentLinkedQueue<String> pubdata = new ConcurrentLinkedQueue<String>();
	public CircularBlockingDeque<Point3f> pointWindow = new CircularBlockingDeque<Point3f>(WINSIZE);
	static enum MODE { HCSR04, URM37};
	static MODE SENSOR_TYPE = MODE.URM37;
	static String portName = null; //auto select, otherwise set to this from command line __PORT:=/dev/ttyxxx
	
	static class EulerTime {
		sensor_msgs.Imu euler;
		long eulerTime = 0L;
	}
	EulerTime euler = new EulerTime();

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("pubs_rangefinder1");
	} 

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		SynchronizedThreadManager.getInstance().init(new String[] {"SYSTEM"}, true);
		Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
		Runnable readThread = null;
		Runnable imuThread = null;
		if( remaps.containsKey("__port") ) {
			portName = remaps.get("__port");
			readThread = new UltraRead();
		} else {
			readThread = new UltraPing();
		}
		if( remaps.containsKey(REMAP_DEBUG) )
			if(remaps.get(REMAP_DEBUG).equals("true")) {
				DEBUG = true;
			}
		final Subscriber<sensor_msgs.Imu> subsimu = connectedNode.newSubscriber("/sensor_msgs/Imu", sensor_msgs.Imu._TYPE);
		imuThread = new IMURead(subsimu);
		SynchronizedThreadManager.getInstance().spin(readThread, "SYSTEM");
		SynchronizedThreadManager.getInstance().spin(imuThread, "SYSTEM");
		// tell the waiting constructors that we have registered publishers if we are intercepting the command line build process
		awaitStart.countDown();
		// This CancellableLoop will be canceled automatically when the node shuts
		// down.
		connectedNode.executeCancellableLoop(new CancellableLoop() {
			final Publisher<std_msgs.String> statpub  = connectedNode.newPublisher("/sensor_msgs/range", std_msgs.String._TYPE);
			@Override
			protected void setup() {

			}

			@Override
			protected void loop() throws InterruptedException {
				try {
					awaitStart.await();
				} catch (InterruptedException e) {}
				if( !pubdata.isEmpty()) {
					String sDist = pubdata.poll();
					if(sDist != null) {
						std_msgs.String statmsg = statpub.newMessage();
						statmsg.setData(sDist);
						statpub.publish(statmsg);
						if(DEBUG)
							System.out.println("Publish:"+sDist);
					}
				}
				Thread.sleep(10);
			}
		});
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
				distance = RangeFinderPubs.getRangeHCSR04(rangefindertrigger, rangefinderresult);
				break;
			case URM37:
				distance = RangeFinderPubs.getRangeURM37(rangefindertrigger, rangefinderresult);
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

	class IMURead implements Runnable {
		public volatile boolean shouldRun = true;
		Subscriber<sensor_msgs.Imu> subsimu;
		public IMURead(Subscriber<sensor_msgs.Imu> subs) {
			subsimu = subs;
		}
		@Override
		public void run() {
			//
			// update all TimedImage in the queue that match the current timestamp to 1 ms with the current
			// IMU reading
			//
			subsimu.addMessageListener(new MessageListener<sensor_msgs.Imu>() {
				@Override
				public void onNewMessage(sensor_msgs.Imu message) {
					if(DEBUG)
						System.out.println("IMU message:"+message.toString());
					synchronized(euler) {
						euler.euler = message;
						euler.eulerTime = System.currentTimeMillis();
					}
				}
			});
			while(shouldRun) {
				try {
					Thread.sleep((int)1e+6);
				} catch (InterruptedException e) {
					shouldRun = false;
				}
			}
		}
	}
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
						distance = RangeFinderPubs.getRangeHCSR04(rangefindertrigger, rangefinderresult);
						break;
					case URM37:
						distance = RangeFinderPubs.getRangeURM37(rangefindertrigger, rangefinderresult);
						break;
					default:
						break;
					}
					//if( distance < MAX_RANGE) { 
					if( DEBUG ) {
						System.out.println("RangeFinder result ="+distance);//(Math.abs(distance-previousDistance)) +" cm");
					}
					pubdata.add(String.valueOf(distance));
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

	class UltraRead implements Runnable {
		public volatile boolean shouldRun = true;	
		@Override
		public void run() {
			UltrasonicSerialDataPort usdp = null;
			if(portName == null)
				usdp = UltrasonicSerialDataPort.getInstance();
			else
				try {
					usdp = UltrasonicSerialDataPort.getInstance(portName);
				} catch (IOException e) {
					e.printStackTrace();
				}

			while(shouldRun) {
				try {
					// Get the range
					double distance = usdp.readDistance();
					//if( distance < MAX_RANGE) { // 500 cm for URM37
					if( DEBUG ) {
						System.out.println("RangeFinder result ="+distance);//(Math.abs(distance-previousDistance)) +" cm");
					}
					synchronized(euler) {
					if(euler.euler != null) {
					double x = Math.sin(euler.euler.getCompassHeadingDegrees()*0.01745329)*distance;
					double y = Math.cos(euler.euler.getCompassHeadingDegrees()*0.01745329)*distance;
					Point3f winPoint = new Point3f((float)x,(float)y,(float)euler.eulerTime);
					pointWindow.add(winPoint);
					if(pointWindow.size() == WINSIZE) {
						ComputeVariance c = new ComputeVariance();
						c.leastVariance(pointWindow);
						StringBuilder sb = new StringBuilder();
						sb.append("Confidence=");
						sb.append(c.getConfidence());
						sb.append(",");
						sb.append("variance1=");
						sb.append(c.getVariance1());
						sb.append(",");
						sb.append("variance2=");
						sb.append(c.getVariance2());
						sb.append(",");
						sb.append("variance3=");
						sb.append(c.getVariance3());
						sb.append(",");
						sb.append("eigen1=");
						sb.append(c.getEigvec1());
						sb.append(",");
						sb.append("eigen2=");
						sb.append(c.getEigvec2());
						sb.append(",");
						sb.append("eigen3=");
						sb.append(c.getEigvec3());
						sb.append("\r\n");
						pubdata.add(String.valueOf(sb.toString()));
					}  
					} else {
						if(DEBUG)
							System.out.println("euler is null..");
					}
					}
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
