package com.neocoretechs.robocore.serialreader.marlinspikeport;

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
	private static boolean DEBUG = true;
	private static volatile PCInterrupts pcInterrupts = null;
	PinState[] PCintMode = new PinState[28];
	double[] PCintValue = new double[2];
	GpioPinDigitalInput[] pins = new GpioPinDigitalInput[28];
	GpioPinAnalogInput[] apins = new GpioPinAnalogInput[2];
	ConcurrentHashMap<Pin, InterruptService> PCintFunc = new ConcurrentHashMap<Pin, InterruptService>();
	volatile int[] PCintLast = new int[3];
	// PlatformManager.setPlatform(Platform.ODROID);
	private GpioController gpio = Pins.gpioController;//GpioFactory.getInstance();
	public static PCInterrupts getInstance() {
		if( pcInterrupts == null )
			synchronized(PCInterrupts.class) {
				if(pcInterrupts == null) {
					pcInterrupts = new PCInterrupts();
				}
			}
		return pcInterrupts;
	}
	
	private PCInterrupts() {
	}

	public void attachInterrupt(int pin, InterruptService userFunc, double value) {
		Pin pipin = null;
		switch(pin) {
			case 37:
				PCintValue[1] = value;
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.AIN1);  // default pin if no pin argument found
				apins[1] = gpio.provisionAnalogInputPin(pipin, String.valueOf(pin));
				apins[1].addListener(this);
				break;
			case 40:
				PCintValue[0] = value;
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.AIN0);  // default pin if no pin argument found
				apins[0] = gpio.provisionAnalogInputPin(pipin, String.valueOf(pin));
				apins[0].addListener(this);
				break;
			default:
				throw new RuntimeException("Analog pin values limited to 37, 40 for AIN1, AIN0 to attach interrupt, but got pin:"+pin);
		}
		PCintFunc.put(pipin,userFunc);	
	}
	
	public void attachInterrupt(int pin, InterruptService userFunc, PinState mode) {
		if(pin >= PCintMode.length)
			throw new RuntimeException("Pin number "+pin+" out of range to provision as digital input to attach interrupt, are you trying to attach a digital pin state to an analog pin?");
		PCintMode[pin] = mode;
		Pin pipin = Pins.getPin(pin);
		PCintFunc.put(pipin,userFunc);
		pins[pin] = gpio.provisionDigitalInputPin(pipin,String.valueOf(pin));
		pins[pin].addListener(this);

	}

	public void detachDigitalInterrupt(int pin) {
		Pin pipin = Pins.getPin(pin);
		PCintFunc.remove(pipin);
		pins[pin].removeListener(this);
		pins[pin] = null;
	}
	
	public void detachAnalogInterrupt(int pin) {
		Pin pipin = null;
		switch(pin) {
			case 37:
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.AIN1);  // default pin if no pin argument found
				apins[1].removeListener(this);
				apins[1] = null;
				break;
			case 40:
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.AIN0);  // default pin if no pin argument found
				apins[0].removeListener(this);
				apins[0] = null;
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
		if(event.getValue() >= PCintValue[pin]) {
			// display pin state on console
			if(DEBUG )
				System.out.println(" --> Analog PIN STATE CHANGE: " + event.getPin() + " = "
	                + event.getValue() +" from pin "+ pin +" linked to interrupt service "+ints+ " with analog value "+PCintValue[pin]);
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
