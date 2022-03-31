package com.neocoretechs.robocore.serialreader.marlinspikeport;

import java.util.Arrays;

import com.pi4j.io.gpio.GpioController;
import com.pi4j.io.gpio.GpioFactory;
import com.pi4j.io.gpio.GpioPin;
import com.pi4j.io.gpio.GpioPinAnalogInput;
import com.pi4j.io.gpio.GpioPinDigitalInput;
import com.pi4j.io.gpio.GpioPinDigitalOutput;
import com.pi4j.io.gpio.OdroidC1Pin;
import com.pi4j.io.gpio.OdroidGpioProvider;
import com.pi4j.io.gpio.Pin;
import com.pi4j.io.gpio.PinMode;
import com.pi4j.io.gpio.PinState;
import com.pi4j.platform.Platform;
import com.pi4j.platform.PlatformAlreadyAssignedException;
import com.pi4j.platform.PlatformManager;
import com.pi4j.util.CommandArgumentParser;
/**
 * If we are not using the default Raspberry Pi platform, we should
 * explicitly assign the platform as the Odroid platform.<p/>
 * Note that some pins, such as pin 7, will give a device not found or device busy runtime exception
 * when provisioning. These pins are, for whatever reason, inaccessible even with command line tools.<p/>
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
public class Pins {
	public static boolean DEBUG = true;
	private static GpioController gpioController = null;
	static GpioPinDigitalInput[] pinsIn = new GpioPinDigitalInput[28];
	static GpioPinDigitalOutput[] pinsOut = new GpioPinDigitalOutput[28];
	static GpioPinAnalogInput[] apins = new GpioPinAnalogInput[2];
	public static double MAXVOLTS = 5.0; //normally analog in is limited to 1.8 volts, but if you are using a level shifter (as you should) it can be up to 5
	/**
	 * Get the logical pin from the physical header pin
	 * @param pin
	 * @return
	 */
	public static Pin getPin(int pin) {
		if(gpioController == null) {
			try {
				PlatformManager.setPlatform(Platform.ODROID);
			} catch (PlatformAlreadyAssignedException e) {
				throw new RuntimeException(e);
			}
			gpioController = GpioFactory.getInstance();
			// <OPTIONAL>
	        // We can optionally override the default polling rate for the analog input monitoring (DEFAULT=50ms)
	        // ... this is the rate at which the internal analog input monitoring thread will poll for changes
	        OdroidGpioProvider.setAnalogInputPollingRate(100); // milliseconds
	        // <OPTIONAL>
	        // We can optionally override the default listener value change threshold (DEFAULT=0)
	        // ... this is the threshold delta value that the internal analog input monitoring thread must cross before
	        //     dispatching a new analog input value change event
	        // analog value must change in excess of 511 from the last event dispatched before dispatching a new change event
	        OdroidGpioProvider.setAnalogInputListenerChangeThreshold(511);
		}
		//Pin[] allPins = OdroidC1Pin.allPins();
		//for(Pin p: allPins)
			//System.out.println("Pin "+p);
		Pin pipin = null;
	    // ####################################################################
        //
        // IF YOU ARE USING AN ODROID C1/C1+ PLATFORM, THEN ...
        //    When provisioning a pin, use the OdroidC1Pin class.
        //
        // ####################################################################
		switch(pin) {
        // lookup the pin by address
			case 0:
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.GPIO_00);  // default pin if no pin argument found
				break;
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
			case 37:
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.AIN1);  // default pin if no pin argument found
				break;
			case 40:
				pipin = CommandArgumentParser.getPin(
		                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
		                OdroidC1Pin.AIN0);  // default pin if no pin argument found
				break;
			default:
				throw new RuntimeException("Digital Pins limited to numbers 0,1,2,3,4,5,6,7 10,11,12,13,14 21,22,23,24 26,27 and analog in 37,40");
		}
		return pipin;
	}
	/**
	 * Assign pin as output
	 * @param pin
	 * @return
	 */
	public static GpioPinDigitalOutput assignPin(int pin) {
		Pin xpin = getPin(pin);
		GpioPinDigitalOutput opin = gpioController.provisionDigitalOutputPin(xpin,String.valueOf(pin),PinState.LOW);
		//opin.setMode(PinMode.DIGITAL_OUTPUT);
		opin.setShutdownOptions(false, PinState.LOW); // false = unexport
		pinsOut[pin] = opin;
		if(DEBUG)
			System.out.printf("Pins.assignPin output pin set %d to %s%n", pin, opin);
		return opin;
	}
	
	public static void unassignPin(GpioPin pin) {
		pin.removeAllListeners();
		gpioController.unprovisionPin(pin);
	}
	
	public static GpioPinDigitalInput assignInputPin(int pin) {
		Pin xpin = getPin(pin);
		GpioPinDigitalInput ipin = gpioController.provisionDigitalInputPin(xpin,String.valueOf(pin));
		//ipin.setMode(PinMode.DIGITAL_INPUT);
		pinsIn[pin] = ipin;
		if(DEBUG)
			System.out.printf("Pins.assignInputPin input pin set %d to %s%n", pin, ipin);
		return ipin;
	}
	
	public static GpioPinAnalogInput assignAnalogInputPin(int pin) {
			Pin pipin = null;
			GpioPinAnalogInput apin;
			if(gpioController == null) {
				try {
					PlatformManager.setPlatform(Platform.ODROID);
				} catch (PlatformAlreadyAssignedException e) {
					throw new RuntimeException(e);
				}
				gpioController = GpioFactory.getInstance();
				// <OPTIONAL>
		        // We can optionally override the default polling rate for the analog input monitoring (DEFAULT=50ms)
		        // ... this is the rate at which the internal analog input monitoring thread will poll for changes
		        OdroidGpioProvider.setAnalogInputPollingRate(100); // milliseconds
		        // <OPTIONAL>
		        // We can optionally override the default listener value change threshold (DEFAULT=0)
		        // ... this is the threshold delta value that the internal analog input monitoring thread must cross before
		        //     dispatching a new analog input value change event
		        // analog value must change in excess of 511 from the last event dispatched before dispatching a new change event
		        OdroidGpioProvider.setAnalogInputListenerChangeThreshold(511);
			}
			switch(pin) {
				case 37:
					pipin = CommandArgumentParser.getPin(
			                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
			                OdroidC1Pin.AIN1);  // default pin if no pin argument found
					apin = gpioController.provisionAnalogInputPin(pipin, String.valueOf(pin));
					break;
				case 40:
					pipin = CommandArgumentParser.getPin(
			                OdroidC1Pin.class,    // pin provider class to obtain pin instance from
			                OdroidC1Pin.AIN0);  // default pin if no pin argument found
					apin =  gpioController.provisionAnalogInputPin(pipin, String.valueOf(pin));
					break;
				default:
					throw new RuntimeException("Analog pin values limited to 37, 40 for AIN1, AIN0, but got pin:"+pin);
			}
			//apin.setMode(PinMode.ANALOG_INPUT);
			if(DEBUG)
				System.out.printf("Pins.assignAnalogInputPin input pin set %d to %s%n", pin, apin);
			return apin;
	}
	
	public static GpioPinDigitalInput getInputPin(int pin) {
		return pinsIn[pin];
	}
	
	public static GpioPinDigitalOutput getOutputPin(int pin) {
		return pinsOut[pin];
	}
	
	public static GpioPinAnalogInput getAnalogInputPin(int pin) {
		switch(pin) {
		case 37:
			return apins[1];
		case 40:
			return apins[0];
		default:
			throw new RuntimeException("Analog pin values limited to 37, 40 for AIN1, AIN0, but got pin:"+pin);
		}
	}

	public String toString() {
		StringBuilder sb = new StringBuilder();
		sb.append("Input pins:");
		sb.append(Arrays.toString(pinsIn));
		sb.append("\r\n");
		sb.append("Output pins:");
		sb.append(Arrays.toString(pinsOut));
		sb.append("\r\n");
		sb.append("Analog Input pins:");
		sb.append(Arrays.toString(apins));
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
