package com.neocoretechs.robocore.serialreader.marlinspikeport.control;

import java.io.IOException;

import com.neocoretechs.robocore.serialreader.marlinspikeport.Pins;
import com.pi4j.io.gpio.GpioPinDigital;
import com.pi4j.io.gpio.GpioPinDigitalOutput;
/**
*SwitchBridgeDriver:
* A motor driver that uses straight GPIO switching rather than PWM, for an on/off drive motor setup.<p/>
*
* Structure:<p/>
* 1) Top level, a master list of pins in the digitals array. Slots to which physical pins are assigned.<p/>
* 2) In each class and subclass, a sublist of pins. Multidimensional array which holds the index to the top level pins.
* these are indexed by channel. motorDrive here.<p/>
* Since the M1 and M2 inputs are split, we add an additional third element in the multidimensional array for the second input.
* 3) A GPIO pin list indexed by channel. The level value is either a high or low value for on or off.<p/>
* 4) Optionally, a duration. The duration represents a maximum interval before the GPIO pin
* is automatically shut down. In this case, the hall effect interrupts that are serviced by an interrupt handler that detect wheel rotation.
* the top level abstract class AbstractMotorControl contains these values.<p/>
* @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2022
*
*/
public class SwitchBridgeDriver extends AbstractMotorControl {
	final static String MSG_MOTORCONTROL_5= "Emergency stop";
	// drive wheel index is by channel-1.
	// motorDrive[channel] [[Digitals array index][dir pin]
	// 0-pin index to Digital pins array(default 255)
	// 1-direction pin
	int[][] motorDrive= new int[][]{{255,255,0},{255,255,0},{255,255,0},{255,255,0},{255,255,0},{255,255,0},{255,255,0},{255,255,0},{255,255,0},{255,255,0}};
	int status_flag = 0;
	
	public int getMotorDigitalPin(int channel) { return motorDrive[channel-1][0]; }
	
	public int getMotorDigitalPinB(int channel) { return motorDrive[channel-1][1]; }

	public int getMotorEnablePin(int channel) {return motorDrive[channel-1][2]; }
	
	public void setMotorDigitalPin(int channel, int pin) { motorDrive[channel-1][0] = pin; }
	
	public void setMotorDigitalPinB(int channel, int pin) { motorDrive[channel-1][1] = pin; }

	public void setMotorEnablePin(int channel, int pin) { motorDrive[channel-1][2] = pin; }
	
	/**
	 * Add a new Digital pin instance to this motor controller.
	 * @param channel the controller channel from 1 to channels
	 * @param pin_number the index in the GPIO pin array defined in 'setMotors' for M1
	 * @param pin_numberB the index in the GPIO array defined in 'setMotors' for M2
	 * @param dir_pin the enable pin for this channel
	 * @param dir_pinB enable for B
	 */
	public void createDigital(int channel, int pin_number, int pin_numberB, int dir_pin, int dir_default) {
		Pins.assignPin(pin_number);
		setMotorDigitalPin(channel, pin_number);
		Pins.assignPin(pin_numberB);
		setMotorDigitalPinB(channel, pin_numberB);
		Pins.assignPin(dir_pin);
		setMotorEnablePin(channel, dir_pin);			
		currentDirection[channel-1] = dir_default;
		defaultDirection[channel-1] = dir_default;
	}
	
	@Override
	public int queryFaultFlag() { return fault_flag; }
	@Override
	public int queryStatusFlag() { return status_flag; }
	
	@Override
	/**
	 * Power either 0 or 1, gpio is on/off, so enable/low, forward, enable/high back
	 * @throws IOException 
	 */
	public int commandMotorPower(int channel, int motorPower) throws IOException {
		// check shutdown override
		if( MOTORSHUTDOWN )
			return 0;
		setMotorSpeed(channel, motorPower == 0 ? 0 : (motorPower < 0 ? -MAXMOTORPOWER : MAXMOTORPOWER));
		int gioIndex = getMotorDigitalPin(channel); // index to gpio array
		int gioIndexB = getMotorDigitalPinB(channel); // index to gpio array
		int dirPinIndex = getMotorEnablePin(channel); // index to dir pin array
		Pins.getOutputPin(dirPinIndex).high(); // enable motor inputs
		//int freq = motorDrive[channel][2]; // value of freq, no index;
		// get mapping of channel to pin
		// see if we need to make a direction change, check array of [PWM pin][dir pin][dir]
		if( getCurrentDirection(channel) == 1) { // if dir 1, we are going what we define as 'forward' 
			if( motorPower < 0 ) { // and we want to go backward
				// reverse dir, send dir change to pin
				// default is 0 (LOW), if we changed the direction to reverse wheel rotation call the opposite dir change signal
				if(getDefaultDirection(channel) > 0) {
					Pins.getOutputPin(gioIndexB).low();
					Pins.getOutputPin(gioIndex).high();
				} else { 
					Pins.getOutputPin(gioIndex).low();
					Pins.getOutputPin(gioIndexB).high();
				}
				setCurrentDirection(channel, 0); // set new direction value
				motorPower = -motorPower; //setMotorSpeed(channel,-motorPower); // absolute val
			}
		} else { // dir is 0
			if( motorPower > 0 ) { // we are going 'backward' as defined by our initial default direction and we want 'forward'
				// reverse, send dir change to pin
				/// default is 0 (HIGH), if we changed the direction to reverse wheel rotation call the opposite dir change signal
				if(getDefaultDirection(channel) > 0) { 
					Pins.getOutputPin(gioIndex).low();
					Pins.getOutputPin(gioIndexB).high();
				} else {
					Pins.getOutputPin(gioIndexB).low();
					Pins.getOutputPin(gioIndex).high();
				}
				setCurrentDirection(channel, 1);
			} else { // backward with more backwardness
				// If less than 0 take absolute value, if zero dont play with sign
				if( motorPower < 0) motorPower = -motorPower; //setMotorSpeed(channel,-motorPower); // absolute val;
			}
		}

		// Reset encoders on new speed setting
		resetEncoders();
		// If we have a linked distance sensor. check range and possibly skip
		// If we are setting power 0, we are stopping anyway
		if(checkUltrasonicShutdown()) {
			// find the pin and get the object we set up in M3 to write to power level
			// element 0 of motorDrive has index to PWM array
			// writing power 0 sets mode 0 and timer turnoff
			setMotorShutdown(); // sends commandEmergencyStop(1);
			return fault_flag;
		}
		fault_flag = 0;
		return 0;
	}

	@Override
	public int commandEmergencyStop(int status) throws IOException {
		for(int j=1; j <= channels; j++) {
			if(getMotorDigitalPin(j) != 255) {
				Pins.getOutputPin(getMotorEnablePin(j)).low();
				Pins.getOutputPin(getMotorDigitalPin(j)).low();
				Pins.getOutputPin(getMotorDigitalPinB(j)).low();
			}
		}
		fault_flag = 16;
		resetSpeeds();
		resetEncoders();
		return status;
	}

	@Override
	public int isConnected() {
		return 1;
	}

	@Override
	public String getDriverInfo(int ch) {
		if( getMotorDigitalPin(ch) == 255 ) {
			return String.format("SwitchBridge UNINITIALIZED Channel %d%n",ch);
		}
		return String.format("SwitchBridge Channel %d PinA:%d, PinB:%d Dir Pin:%d%n",ch, getMotorDigitalPin(ch), getMotorDigitalPinB(ch), getMotorEnablePin(ch));	
	}
	
	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder();
		for(int i = 1; i <= channels; i++) {
			if(motorDrive[i-1][0] != 255)
				sb.append(getDriverInfo(i));
		}
		return sb.toString();
	}

	@Override
	public String getMotorFaultDescriptor(int fault) {
		if(fault != 0)
			return MSG_MOTORCONTROL_5;
		return "";
	}

	@Override
	public String getMotorStatusDescriptor(int status) {
		return "";
	}

	@Override
	/**
	 * To service a digital pin with an interrupt we will have to wrap it to implement InterruptServiceHandlerInterface
	 * as we do the PWM pin
	 */
	public void setInterruptServiceHandler(int intPin) {
		
	}

}
