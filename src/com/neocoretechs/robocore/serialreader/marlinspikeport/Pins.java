package com.neocoretechs.robocore.serialreader.marlinspikeport;

import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.ConcurrentHashMap;

import com.neocoretechs.robocore.GpioNative;
/**
 * Conforms to new libgpiod protocols. {@link GpioNative} 
 * explicitly assume the platform as the Odroid platform.<p>
 * The 'pins' referred to here are the GPIO pin designations which map to the physical 40 pin header as follows:<p>
 * Phy | <br>
 * 1  | 1		<br>
 * 2  | 2		<br>
 * 3  | 3		<br>
 * 4  | 4		<br>
 * 5  | 5		<br>
 * 6  | 6		<br>
 * 7  | 7	(PWM_C pwmchip0/pwm0)	<br>
 * 8  | 8		<br>
 * 9  | 9		<br>
 * 10 | 10		<br>
 * 11 | 11		<br>
 * 12  | 12	(PWM_E pwmchip4/pwm0)	<br>
 * 13  | 13		<br>
 * 14  | 14		<br>
 * 15  | 15		<br>
 * 16  | 16	(PIN_16 gpiochip1 offset 66)	<br>
 * 17  | 17		<br>
 * 18  | 18 (PIN_18 gpiochip1 offset 67)		<br>
 * 19  | 19		<br>
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2022
 *
 */
public class Pins {
	public static boolean DEBUG = false;
	public static int pinsIn[] = new int[80];
	public static int pinsOut[] = new int[80];
	public static ConcurrentHashMap<String, Integer> Pin = new ConcurrentHashMap<String,Integer>();
	public static int err;
	
	public static GpioNative gpio = new GpioNative();
	public static String gpioChip = "gpiochip1";
	public static boolean chipOpen = false;
	static int analogPollRate = 100; // ms between analog change for firing
	static int analogChangeThreshold = 511; // amount analog input has to change before firing
	public static double MAXVOLTS = 5.0; //normally analog in is limited to 1.8 volts, but if you are using a level shifter (as you should) it can be up to 5
	/**
	 * Get the logical pin from the physical header pin
	 * @param pin
	 * @return
	 */
	private static int getPin(String spin) throws IOException {
		if(!chipOpen) {
		  	if((err = gpio.openChip(gpioChip)) < 0)
	    		throw new IOException("chipOpen error "+err);	
			chipOpen = true;
		}
	   	// get handle to line struct
		Integer ipin = Pin.get(spin);
		if(ipin == null) {
			ipin = gpio.findChipLine(spin);
			if(ipin < 0)
				throw new IOException("findChipLine error:"+ipin);
			Pin.put(spin, ipin);
		}
    	return ipin;
	}
	/**
	 * Assign pin as output
	 * @param pin
	 * @return actual pin handle
	 * @throws IOException 
	 */
	private static int assignOutputPin(int ipin) throws IOException {
		int pin = getPin("PIN_"+ipin);
		if(pinsOut[ipin] == 0) {
			if((err = gpio.lineRequestOutput(pin)) < 0)
				throw new IOException("lineRequestOutput error:"+err);
			pinsOut[ipin] = pin;
			if(DEBUG)
				System.out.printf("Pins.assignPin output pin set %d%n", pin);
		}
		return pin;
	}
	
	public static void unassignPin(int ipin) throws IOException {
		Integer gpin = Pin.get("PIN_"+ipin);
		if(gpin != null) {
			if((err = gpio.lineRelease(gpin)) < 0)
				throw new IOException("unassignPin error:"+err);
			Pin.remove("PIN_"+ipin);
	  		pinsIn[gpin] = 0;
	  		pinsOut[gpin] = 0;
		}
	}
	
	private static int assignInputPin(int ipin) throws IOException {
		int pin = getPin("PIN_"+ipin);
		if(pinsIn[ipin] == 0) {
			if((err = gpio.lineRequestInput(pin)) < 0)
				throw new IOException("lineRequestOutput error:"+err);
			pinsIn[ipin] = pin;
			if(DEBUG)
				System.out.printf("Pins.assignInputPin input pin set %d%n", pin);
		}
		return pin;
	}
	
	private static int assignAnalogInputPin(int ipin) throws IOException {
		int pin = getPin("PIN_"+ipin);
		if(pinsIn[ipin] == 0) {
			if((err = gpio.lineRequestRisingEdgeEvents(pin)) < 0)
				throw new IOException("lineRequestRisingEdgeEvents error:"+err);
			pinsIn[pin] = pin;
		}
		return pin;
	}
	
	public static int getInputPin(int ipin) throws IOException {
		int pin = assignInputPin(ipin);
		int ret = gpio.lineGetValue(pin);
		if(ret < 0)
			throw new IOException("lineGetValue error:"+ret);
		return ret;
	}
	
	public static void getOutputPin(int ipin, int val) throws IOException{
		int pin = assignOutputPin(ipin);
		int ret = gpio.lineSetValue(pin, val);
		if(ret < 0)
			throw new IOException("lineSetValue error:"+ret);
	}
	/**
	 * 
	 * @param pin
	 * @return 1 rising edge, 3 falling edge, 0 unknown
	 * @throws IOException
	 */
	public static int getAnalogInputPin(int ipin) throws IOException {
		int pin = assignAnalogInputPin(ipin);
		while((err = gpio.lineEventWait(pin)) == 0) {
			try {
				Thread.sleep(10);
			} catch (InterruptedException e) {}
		}
		if(err < 0)
			throw new IOException("lineEventWait error:"+err);
		int ret = gpio.lineEventRead(pin);
		if(ret < 0)
			throw new IOException("lineEventRead error:"+ret);
		return ret;
		/*
		switch(pin) {
		case 37:
			return apins[1];
		case 40:
			return apins[0];
		default:
			throw new RuntimeException("Analog pin values limited to 37, 40 for AIN1, AIN0, but got pin:"+pin);
		}
		*/
	}
	
	public static void setAnalogPollRate(int rate) {
		analogPollRate = rate;
	}
	
	public static void setAnalogChangeDelta(int delta) {
		analogChangeThreshold = delta;
	}
	
	public String toString() {
		StringBuilder sb = new StringBuilder();
		sb.append("Input pins:");
		sb.append(Arrays.toString(pinsIn));
		sb.append("\r\n");
		sb.append("Output pins:");
		sb.append(Arrays.toString(pinsOut));
		sb.append("\r\n");
		//sb.append("Analog Input pins:");
		//sb.append(Arrays.toString(apins));
		sb.append("\r\n");
		return sb.toString();
	}
	 /**
     * calculate relative analog input voltage based on the 10-bit conversion value
     * read from the hardware
     *
     * @param value 10-bit conversion value for analog input pin
     * @return relative voltage for analog input pin
     */
    private static double getVoltage(double value){
        // 10-bit == range between 0 and 1023 (1024 possible values)
        return (value / 1024) * MAXVOLTS; // 1.8VDC maximum allowed voltage per the hardware spec
    }
}
