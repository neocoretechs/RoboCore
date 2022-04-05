package com.neocoretechs.robocore.serialreader.marlinspikeport;

import java.util.Arrays;
import java.util.concurrent.ConcurrentHashMap;

import com.pi4j.io.gpio.GpioPinAnalogInput;
import com.pi4j.io.gpio.GpioPinDigitalInput;
import com.pi4j.io.gpio.OdroidC1Pin;
import com.pi4j.io.gpio.Pin;
import com.pi4j.io.gpio.PinState;
import com.pi4j.io.gpio.event.GpioPinAnalogValueChangeEvent;
import com.pi4j.io.gpio.event.GpioPinDigitalStateChangeEvent;
import com.pi4j.io.gpio.event.GpioPinListenerAnalog;
import com.pi4j.io.gpio.event.GpioPinListenerDigital;
import com.pi4j.util.CommandArgumentParser;
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
public class PCInterrupts implements GpioPinListenerDigital, GpioPinListenerAnalog {
	private static boolean DEBUG = true;
	static PinState[] PCintMode = new PinState[28];
	static double[] PCintLoValue = new double[2];
	static double[] PCintHiValue = new double[2];
	static ConcurrentHashMap<Pin, InterruptService> PCintFunc = new ConcurrentHashMap<Pin, InterruptService>();
	/**
	 * Attach analog input pin to state change interrupt
	 * @param pin
	 * @param userFunc
	 * @param value
	 */
	public void attachInterrupt(int pin, InterruptService userFunc, double loValue, double hiValue) {
		GpioPinAnalogInput ppin = Pins.assignAnalogInputPin(pin);
		ppin.addListener(this);
		Pin pipin = Pins.getPin(pin);
		switch(pin) {
			case 37:
				Pins.apins[1] = ppin;
				PCintLoValue[1] = loValue;
				PCintHiValue[1] = hiValue;
				break;
			case 40:
				Pins.apins[0] = ppin;
				PCintLoValue[0] = loValue;
				PCintHiValue[0] = hiValue;
				break;
			default:
				throw new RuntimeException("Analog pin values limited to 37, 40 for AIN1, AIN0 to attach interrupt, but got pin:"+pin);
		}
		PCintFunc.put(pipin,userFunc);	
	}
	
	/**
	 * Attach digital input pin to state change interrupt
	 * @param pin
	 * @param userFunc
	 * @param mode
	 */
	public void attachInterrupt(int pin, InterruptService userFunc, PinState mode) {
		if(pin >= PCintMode.length)
			throw new RuntimeException("Pin number "+pin+" out of range to provision as digital input to attach interrupt, are you trying to attach a digital pin state to an analog pin?");
		PCintMode[pin] = mode;
		Pin pipin = Pins.getPin(pin);
		PCintFunc.put(pipin,userFunc);
		Pins.pinsIn[pin] = Pins.assignInputPin(pin);
		Pins.pinsIn[pin].addListener(this);

	}
	
	/**
	 * Detach digital interrupt from pin
	 * @param pin
	 */
	public void detachDigitalInterrupt(int pin) {
		Pin pipin = Pins.getPin(pin);
		PCintFunc.remove(pipin);
		Pins.pinsIn[pin].removeListener(this);
		Pins.pinsIn[pin] = null;
	}
	
	/**
	 * Detach analog interrupt from analog input pin
	 * @param pin
	 */
	public void detachAnalogInterrupt(int pin) {
		Pin pipin = null;
		switch(pin) {
			case 37:
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.AIN1);  // default pin if no pin argument found
				Pins.apins[1].removeListener(this);
				Pins.apins[1] = null;
				break;
			case 40:
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.AIN0);  // default pin if no pin argument found
				Pins.apins[0].removeListener(this);
				Pins.apins[0] = null;
				break;
			default:
				throw new RuntimeException("Analog pin values limited to 37, 40 for AIN1, AIN0 to detach interrupt, but got pin:"+pin);
		}
		PCintFunc.remove(pipin);	
	}
	
	@Override
	public void handleGpioPinDigitalStateChangeEvent(GpioPinDigitalStateChangeEvent event) {
		InterruptService ints =  PCintFunc.get(event.getPin().getPin());
		if(ints == null)
			throw new RuntimeException("Digital Pin "+event.getPin().getPin()+" returned null, error in provisioning");
		int pin = ints.getPin();
	     // Trigger interrupt if mode is CHANGE, or if mode is RISING and
	     // the bit is currently high, or if mode is FALLING and bit is low.
		if ( event.getState().equals(PCintMode[pin])) {
			// display pin state on console
			if(DEBUG )
				System.out.println(" --> Digital GPIO PIN STATE CHANGE: " + event.getPin() + " = "
	                + event.getState()+" from pin "+ pin +" linked to interrupt service "+ints+ " with digital state "+PCintMode[pin]);
			PCintFunc.get(event.getPin().getPin()).service();
		}
	}

	@Override
	public void handleGpioPinAnalogValueChangeEvent(GpioPinAnalogValueChangeEvent event) {
		InterruptService ints =  PCintFunc.get(event.getPin().getPin());
		if(ints == null)
			throw new RuntimeException("Analog Pin "+event.getPin().getPin()+" returned null, error in provisioning");
		int pin = ints.getPin();
		int ppin;
		switch(pin) {
			case 37:
				ppin = 1;
				break;
			case 40:
				ppin = 0;
				break;
			default:
				throw new RuntimeException("Analog pin values limited to 37, 40 for AIN1, AIN0 to attach interrupt, but got pin:"+pin);
		}
		//if(DEBUG )
		//	System.out.println("POTENTIAL Analog PIN STATE CHANGE: " + event.getPin() + " = "
        //        + event.getValue() +" from pin "+ pin +" linked to interrupt service "+ints+ " with analog values "+PCintLoValue[ppin]+" to "+PCintHiValue[ppin]);
		if(event.getValue() >= PCintLoValue[ppin] && event.getValue() <= PCintHiValue[ppin]) {
			// display pin state on console
			if(DEBUG )
				System.out.println(" --> Analog PIN STATE CHANGE: " + event.getPin() + " = "
	                + event.getValue() +" from pin "+ pin +" linked to interrupt service "+ints+ " with analog values "+PCintLoValue[ppin]+" to "+PCintHiValue[ppin]);
			ints.service();
		}
		
	}
	
	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder("Pin Change Interrupts:\r\n");
		PCintFunc.forEach((k,v) -> sb.append(String.format("Pin:%s Interrupt:%s%n",k,v)));
		return sb.toString();
	}


}
