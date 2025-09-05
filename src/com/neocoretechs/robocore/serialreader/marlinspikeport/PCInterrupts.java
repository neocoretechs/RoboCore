package com.neocoretechs.robocore.serialreader.marlinspikeport;

import java.io.IOException;

import java.util.concurrent.ConcurrentHashMap;

/**
 * The 'pins' referred to here are the WiringPi GPIO pin designations which map to the physical 40 pin header as follows:<p/>
 * Phy | WPi <br/>
 * 11  | 0		<br/>
 * 12  | 1		<br/>
 * 13  | 2		<br/>
 * 15  | 3		<br/>
 * 16  | 4		<br/>
 * 18  | 5		<br/>
 * 22  | 6		<br/>
 * 7   | 7	(unusable Odroid)	<br/>
 * 24  | 10		<br/>
 * 26  | 11		<br/>
 * 19  | 12	(PWM1)	<br/>
 * 21  | 13		<br/>
 * 23  | 14		<br/>
 * 29  | 21		<br/>
 * 31  | 22		<br/>
 * 33  | 23	(PWM0)	<br/>
 * 35  | 24		<br/>
 * 36  | 27		<br/>
 * When designating pins, use the WPi numbers, except for the analog input pins which are designed as: <p>
 * 37  | AIN1	<br/>
 * 40  | AIN0	<br/>
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2022
 *
 */
public class PCInterrupts /*implements GpioPinListenerDigital, GpioPinListenerAnalog*/ {
	private static boolean DEBUG = false;
	//static PinState[] PCintMode = new PinState[28];
	static double[] PCintLoValue = new double[2];
	static double[] PCintHiValue = new double[2];
	static ConcurrentHashMap<Integer, InterruptServiceInterface> PCintFunc = new ConcurrentHashMap<Integer, InterruptServiceInterface>();
	/**
	 * Attach analog input pin to state change interrupt
	 * @param pin
	 * @param userFunc
	 * @param value
	 * @throws IOException 
	 */
	public void attachInterrupt(int pin, InterruptServiceInterface userFunc, double loValue, double hiValue) throws IOException {
		PCintFunc.put(pin,userFunc);	
	}
	
	/**
	 * Attach digital input pin to state change interrupt
	 * @param pin
	 * @param userFunc
	 * @param mode
	 * @throws IOException 
	 */
	public void attachInterrupt(int pin, InterruptServiceInterface userFunc, int mode) throws IOException {
		//if(pin >= PCintMode.length)
		//	throw new RuntimeException("Pin number "+pin+" out of range to provision as digital input to attach interrupt, are you trying to attach a digital pin state to an analog pin?");
		//PCintMode[pin] = mode;
		//Pin pipin = Pins.getPin(pin);
		//Pins.assignAnalogInputPin(pin);
		PCintFunc.put(pin,userFunc);
		//Pins.pinsIn[pin] = Pins.assignInputPin(pin);
		//Pins.pinsIn[pin].addListener(this);
	}
	
	/**
	 * Detach digital interrupt from pin
	 * @param pin
	 * @throws IOException 
	 */
	public void detachDigitalInterrupt(int pin) throws IOException {
		//Pin pipin = Pins.getPin(pin);
		PCintFunc.remove(pin);
		//Pins.pinsIn[pin].removeListener(this);
		Pins.pinsIn[pin] = 0;
		Pins.unassignPin(pin);
	}
	
	/**
	 * Detach analog interrupt from analog input pin
	 * @param pin
	 * @throws IOException 
	 */
	public void detachAnalogInterrupt(int pin) throws IOException {
		PCintFunc.remove(pin);	
		Pins.unassignPin(pin);
	}
	
	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder("Pin Change Interrupts:\r\n");
		PCintFunc.forEach((k,v) -> sb.append(String.format("Pin:%s Interrupt:%s%n",k,v)));
		return sb.toString();
	}


}
