package com.neocoretechs.robocore.ranging;
import java.io.BufferedReader;
import java.io.FileReader;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import org.ros.internal.node.server.ThreadPoolManager;
import com.pi4j.io.gpio.GpioController;
import com.pi4j.io.gpio.GpioFactory;
import com.pi4j.io.gpio.GpioPinDigitalInput;
import com.pi4j.io.gpio.GpioPinDigitalOutput;
import com.pi4j.io.gpio.PinPullResistance;
import com.pi4j.io.gpio.PinState;
import com.pi4j.io.gpio.RaspiPin;
import com.pi4j.io.gpio.event.GpioPinDigitalStateChangeEvent;
import com.pi4j.io.gpio.event.GpioPinListenerDigital;
/**
 * Use the Pi4J GPIO libraries to drive an ultrasonic range finder attached to GPIO pins 2 and 3 on the RasPi.
 * Pin2 being the trigger and pin 3 being the resulting signal.
 * The trigger pulse goes high to low with a 10 microsecond wait.
 * There is a 2 second window where the result pin goes from low to high that we use as our interval
 * for sensing an object.
 * We use a 250 millisecond delay between trigger and result pin provisioning with trigger set to low and result pulled up.
 * From there we enter the loop to acquire the ranging checking for a 300cm maximum distance.
 * If we get something that fits these parameters we publish to the robocore/status bus. We can also shut down
 * publishing to the bus while remaining in the detection loop by sending a message to the alarm/shutdown topic.
 * @author Jonathan Groff (C) NeoCoreTechs 2020
 *
 */
public class RangeFinderPubs  extends AbstractNodeMain {
	private static boolean DEBUG = false;
	static long count = 0;
	static boolean alarmTrip = false;
	double result = 0;
	GpioPinDigitalOutput firepulse;
	GpioPinDigitalInput result_pin;
	private static final int PULSE = 10000;        // #10 us pulse = 10,000 ns
	private static final int SPEEDOFSOUND = 34029; // Speed of sound = 34029 cm/s
	//public static VoxHumana speaker = null;
	public ConcurrentLinkedQueue<String> pubdata = new ConcurrentLinkedQueue<String>();
	private static final int DISTANCE_THRESHOLD = 10; // 10 cm variance
	private static double previousDistance = -1;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("pubs_rangefinder1");
	} 

	@Override
	public void onStart(final ConnectedNode connectedNode) {
		ThreadPoolManager.init(new String[] {"SYSTEM"}, true);
		//final Log log = connectedNode.getLog();
		final Publisher<std_msgs.String> statpub =
				connectedNode.newPublisher("robocore/alerts", std_msgs.String._TYPE);

		UltraPing up = new UltraPing();
		ThreadPoolManager.getInstance().spin(up, "SYSTEM");
		// This CancellableLoop will be canceled automatically when the node shuts
		// down.
		connectedNode.executeCancellableLoop(new CancellableLoop() {
			std_msgs.String statmsg = statpub.newMessage();
			@Override
			protected void setup() {
				
			}

			@Override
			protected void loop() throws InterruptedException {
				if( !pubdata.isEmpty()) {
					String sDist = pubdata.poll();
					if(sDist != null) {
						statmsg.setData(sDist);
						statpub.publish(statmsg);
					}
				}
				Thread.sleep(1);
			}
		});
	}
	
	
	 /**
	  * 
	  * Trigger the Range Finder and return the result
	  * 
	  * @return
	  */
	 public static double getRange(GpioPinDigitalOutput rangefindertrigger,  GpioPinDigitalInput rangefinderresult ) {
	  long distance = -1;
	  try {
		  // fire the trigger pulse, 1 then 0 with 10 microsecond wait 
		  rangefindertrigger.high();
		  Thread.sleep(0, PULSE);// wait 10 us
		  rangefindertrigger.low();
		  long starttime = System.nanoTime(); //ns
	      long stop = starttime;
	      long start = starttime;
	      //echo will go 0 to 1 and need to save time for that. 2 seconds difference
	      while ((rangefinderresult.getState() == PinState.LOW) && (start < starttime + 1000000000L * 2)) {
	          start = System.nanoTime();
	      }
	      while ((rangefinderresult.getState() == PinState.HIGH) && (stop < starttime + 1000000000L * 2)) {
	          stop = System.nanoTime();
	      }
	      long delta = (stop - start);
	      distance = delta * SPEEDOFSOUND; // echo from 0 to 1 depending on object distance
	      //Thread.sleep(20);
	  } catch (InterruptedException e) {    
		  e.printStackTrace();
		  System.out.println("Exception triggering range finder:"+e);
	  }
	 
	  return distance / 2.0 / (1000000000L); // cm/s; 
	 }
	 
	 public static void main(String[] args) {	 
		  // Setup GPIO Pins 
		  GpioController gpio = GpioFactory.getInstance();
		  //range finder pins 
		  GpioPinDigitalOutput rangefindertrigger = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_02, "Range Finder Trigger", PinState.LOW);
		  try {
			Thread.sleep(250);
		  } catch (InterruptedException e) {}
		  GpioPinDigitalInput rangefinderresult = gpio.provisionDigitalInputPin(RaspiPin.GPIO_03, "Range Pulse Result", PinPullResistance.PULL_UP);
		  try {
			Thread.sleep(250);
		  } catch (InterruptedException e) {}
	
		  do {
			  // Get the range
			  double distance=RangeFinderPubs.getRange(rangefindertrigger, rangefinderresult);
			  if(previousDistance == -1)
				  previousDistance = distance;
			  if( Math.abs(distance-previousDistance) >= DISTANCE_THRESHOLD) { // 300 cm, max range is 200 cm typical
				  if( DEBUG ) {
					  System.out.println();
					  System.out.println("RangeFinder result ="+distance +" cm");
				  }
			  }
		  
		  } while (true);
		   	   
	}


class UltraPing implements Runnable {
	public volatile boolean shouldRun = true;
	
	@Override
	public void run() {
		// Setup GPIO Pins 
		GpioController gpio = GpioFactory.getInstance();
		//range finder pins 
		final GpioPinDigitalOutput rangefindertrigger = gpio.provisionDigitalOutputPin(RaspiPin.GPIO_02, "Range Finder Trigger", PinState.LOW);
		try {
			Thread.sleep(250);
		} catch (InterruptedException e) {}
		final GpioPinDigitalInput rangefinderresult = gpio.provisionDigitalInputPin(RaspiPin.GPIO_03, "Range Pulse Result", PinPullResistance.PULL_UP);
		try {
			Thread.sleep(250);
		} catch (InterruptedException e) {}
		while(shouldRun) {
		try {
			 // Get the range
			  double distance=RangeFinderPubs.getRange(rangefindertrigger, rangefinderresult);
			  if(previousDistance == -1)
				  previousDistance = distance;
			  if( Math.abs(distance-previousDistance) >= DISTANCE_THRESHOLD) { // 300 cm, max range is 200 cm typical
				  if( DEBUG ) {
					  System.out.println();
					  System.out.println("RangeFinder result ="+distance +" cm");
				  }
				  if(DEBUG)
					System.out.print("Intruder ALERT!!\r");
				  pubdata.add(String.valueOf(Math.abs(distance-previousDistance)));
			  }
			  previousDistance = distance;
			
		} catch (Exception e) {
			e.printStackTrace();
		}
		}
	}
}

}
