package com.neocoretechs.robocore.ranging;
import javax.sound.sampled.LineUnavailableException;

import com.neocoretechs.robocore.GenerateTone;
import com.pi4j.io.gpio.GpioController;
import com.pi4j.io.gpio.GpioFactory;
import com.pi4j.io.gpio.GpioPinDigitalInput;
import com.pi4j.io.gpio.GpioPinDigitalOutput;
import com.pi4j.io.gpio.PinPullResistance;
import com.pi4j.io.gpio.PinState;
import com.pi4j.io.gpio.RaspiPin;

public class RangeFinder {
		private static boolean DEBUG = false;
		static long count = 0;
		static boolean alarmTrip = false;
		double result = 0;
		GpioPinDigitalOutput firepulse;
		GpioPinDigitalInput result_pin;
		private static final int PULSE = 10000;        // #10 us pulse = 10,000 ns
		private static final int SPEEDOFSOUND = 34029; // Speed of sound = 34029 cm/s
		private static final int DISTANCE_THRESHOLD = 10; // 10 cm variance
		private static double previousDistance = -1;
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
				  double distance=RangeFinder.getRange(rangefindertrigger, rangefinderresult);
				  if(previousDistance == -1)
					  previousDistance = distance;
				  if( Math.abs(distance-previousDistance) >= DISTANCE_THRESHOLD) { // 300 cm, max range is 200 cm typical
					  if( DEBUG ) {
						  System.out.println();
						  System.out.println("RangeFinder result ="+distance +" cm");
					  }
	
						  if(DEBUG)
							  System.out.print("Intruder ALERT!!\r");
						  count = System.currentTimeMillis();
						for(int i = 0; i < 15; i++) {
								try {
									GenerateTone.generateTone(1500, 50, 100, true);
								} catch (LineUnavailableException e1) {
									// TODO Auto-generated catch block
									e1.printStackTrace();
								}
								try {
									Thread.sleep(10);
								} catch (InterruptedException e) {
									e.printStackTrace();
								}
						}
				  }
				  previousDistance = distance;
	
			  } while (true);
			   	   
		}
	}
