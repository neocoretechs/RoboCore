package com.neocoretechs.robocore.serialreader;

import java.util.concurrent.ConcurrentHashMap;

import com.pi4j.io.gpio.GpioController;
import com.pi4j.io.gpio.GpioFactory;
import com.pi4j.io.gpio.GpioPin;
import com.pi4j.io.gpio.GpioPinAnalogInput;
import com.pi4j.io.gpio.GpioPinDigitalInput;
import com.pi4j.io.gpio.OdroidC1Pin;
import com.pi4j.io.gpio.Pin;
import com.pi4j.io.gpio.PinState;
import com.pi4j.io.gpio.event.GpioPinAnalogValueChangeEvent;
import com.pi4j.io.gpio.event.GpioPinDigitalStateChangeEvent;
import com.pi4j.io.gpio.event.GpioPinListenerAnalog;
import com.pi4j.io.gpio.event.GpioPinListenerDigital;
import com.pi4j.platform.Platform;
import com.pi4j.platform.PlatformManager;
import com.pi4j.util.CommandArgumentParser;

public class PCInterrupts implements GpioPinListenerDigital, GpioPinListenerAnalog {
	private static volatile PCInterrupts pcInterrupts = null;
	PinState[] PCintMode = new PinState[27];
	double[] PCintValue = new double[2];
	GpioPinDigitalInput[] pins = new GpioPinDigitalInput[27];
	GpioPinAnalogInput[] apins = new GpioPinAnalogInput[2];
	ConcurrentHashMap<Pin, InterruptService> PCintFunc = new ConcurrentHashMap<Pin, InterruptService>();
	volatile int[] PCintLast = new int[3];
	// PlatformManager.setPlatform(Platform.ODROID);
	private GpioController gpio = null;//GpioFactory.getInstance();
	private boolean DEBUG;
	public static PCInterrupts getInstance(GpioController gpio) {
		if( pcInterrupts == null )
			synchronized(PCInterrupts.class) {
				if(pcInterrupts == null) {
					pcInterrupts = new PCInterrupts(gpio);
				}
			}
		return pcInterrupts;
	}
	
	private PCInterrupts(GpioController gpio) {
		this.gpio = gpio;
	}

	public void attachInterrupt(int pin, InterruptService userFunc, double value) {
		Pin pipin = null;
		switch(pin) {
			case 39:
				PCintValue[1] = value;
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.AIN1);  // default pin if no pin argument found
				apins[1] = gpio.provisionAnalogInputPin(pipin, String.valueOf(pin));
				break;
			case 40:
				PCintValue[0] = value;
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.AIN0);  // default pin if no pin argument found
				apins[0] = gpio.provisionAnalogInputPin(pipin, String.valueOf(pin));
				break;
			default:
				throw new RuntimeException("Analog pin values limited to 39, 40 for AIN1, AIN0");
		}
		PCintFunc.put(pipin,userFunc);	
	}
	
	public void attachInterrupt(int pin, InterruptService userFunc, PinState mode) {
		PCintMode[pin] = mode;
		Pin pipin = null;
	    // ####################################################################
        //
        // IF YOU ARE USING AN ODROID C1/C1+ PLATFORM, THEN ...
        //    When provisioning a pin, use the OdroidC1Pin class.
        //
        // ####################################################################
		switch(pin) {
        // by default we will use gpio pin #01; however, if an argument
        // has been provided, then lookup the pin by address
			case 0:
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.GPIO_00);  // default pin if no pin argument found
			case 1:
				pipin = CommandArgumentParser.getPin(
						OdroidC1Pin.class,    // pin provider class to obtain pin instance from
						OdroidC1Pin.GPIO_01);  // default pin if no pin argument found
				break;
			case 2:
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.GPIO_02);  // default pin if no pin argument found
				break;
			case 3:
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.GPIO_03);  // default pin if no pin argument found
				break;
			case 4:
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.GPIO_04);  // default pin if no pin argument found
				break;
			case 5:
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.GPIO_05);  // default pin if no pin argument found
				break;
			case 6:
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.GPIO_06);  // default pin if no pin argument found
				break;
			case 7:
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.GPIO_07);  // default pin if no pin argument found
				break;
			case 10:
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.GPIO_10);  // default pin if no pin argument found
				break;
			case 11:
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.GPIO_11);  // default pin if no pin argument found
				break;
			case 12:
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.GPIO_12);  // default pin if no pin argument found
				break;
			case 13:
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.GPIO_13);  // default pin if no pin argument found
				break;
			case 14:
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.GPIO_14);  // default pin if no pin argument found
				break;
			case 21:
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.GPIO_21);  // default pin if no pin argument found
				break;
			case 22:
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.GPIO_22);  // default pin if no pin argument found
				break;
			case 23:
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.GPIO_23);  // default pin if no pin argument found
				break;
			case 24:
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.GPIO_24);  // default pin if no pin argument found
				break;
			case 26:
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.GPIO_26);  // default pin if no pin argument found
				break;
			case 27:
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.GPIO_27);  // default pin if no pin argument found
				break;
			default:
				throw new RuntimeException("Digital Pins limited to numbers 0,1,2,3,4,5,6,7 10,11,12,13,14 21,22,23,24 26,27");
		}
		PCintFunc.put(pipin,userFunc);
		pins[pin] = gpio.provisionDigitalInputPin(pipin,String.valueOf(pin));

	}
	public void detachInterrupt(int interruptNum) {
		
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
	                + event.getState()+" from pin "+ pin +" linked to interrupt service "+ints+ " with digital state "+PCintValue[pin]);
			PCintFunc.get(event.getPin().getPin()).service();
		}
	}

	@Override
	public void handleGpioPinAnalogValueChangeEvent(GpioPinAnalogValueChangeEvent event) {
		InterruptService ints =  PCintFunc.get(event.getPin().getPin());
		if(ints == null)
			throw new RuntimeException("Analog Pin "+event.getPin().getPin()+" returned null, error in provisioning");
		int pin = ints.getPin();
		if(event.getValue() == PCintValue[pin]) {
			// display pin state on console
			if(DEBUG )
				System.out.println(" --> Analog PIN STATE CHANGE: " + event.getPin() + " = "
	                + event.getValue() +" from pin "+ pin +" linked to interrupt service "+ints+ " with analog value "+PCintValue[pin]);
			ints.service();
		}
		
	}

}
