package com.neocoretechs.robocore.serialreader.marlinspikeport.control;

import java.io.IOException;

import com.neocoretechs.robocore.serialreader.marlinspikeport.Pins;

/**
* SwitchHBridgeDriver:
* A motor driver that uses straight GPIO switching rather than PWM, for an on/off drive motor setup.<p/>
* The SwitchHBridgeDriver extends and differs from the SwitchBridgeDriver in that it uses one direction pin and one signal
* pin vs the 2 half bridge orientation of {@link SwitchBridgeDriver} that takes an enable pin and 2 signal pins 9one for each half bridge)<p/>
* Structure:<p/>
* 1) Top level, a master list of pins in the digitals array. Slots to which physical pins are assigned.<p/>
* 2) In each class and subclass, a sublist of pins. Multidimensional array which holds the index to the top level pins.
* these are indexed by channel.<p/>
* 3) A GPIO pin list indexed by channel. The level value is either a high or low for on or off.<p/>
* 4) Optionally, a duration. The duration represents a maximum interval before the GPIO pin
* is automatically shut down. In this case, the hall effect interrupts that are serviced by an interrupt handler that detect wheel rotation.
* the top level abstract class AbstractMotorControl contains these values.<p/>
* Created: 10/20/2020 1:39:44 PM
* @author Jonathan Groff Copyright (C) NeoCoreTechs 2022
*
*/
public class SwitchHBridgeDriver extends SwitchBridgeDriver {

	/**
	 * Add a new Digital pin instance to this motor controller.
	 * @param channel the controller channel from 1 to channels
	 * @param pin_number the index in the GPIO pin array defined in 'setMotors' for M1
	 * @param pin_numberB the index in the GPIO array defined in 'setMotors' for M2
	 * @param dir_pin the enable pin for this channel
	 */
	public void createDigital(int channel, int pin_number, int dir_pin, int dir_default) {
		Pins.assignPin(pin_number);
		setMotorDigitalPin(channel, pin_number);
		Pins.assignPin(dir_pin);
		setMotorEnablePin(channel, dir_pin);			
		currentDirection[channel-1] = dir_default;
		defaultDirection[channel-1] = dir_default;
	}
	
	@Override
	public int commandMotorPower(int channel, int motorPower) throws IOException {
		// check shutdown override
		if( MOTORSHUTDOWN )
			return 0;
		setMotorSpeed(channel, motorPower == 0 ? 0 : (motorPower < 0 ? -MAXMOTORPOWER : MAXMOTORPOWER));
		int pIndex = getMotorDigitalPin(channel); // drive pin
		int dirPinIndex = getMotorEnablePin(channel); // index to dir pin array
		// get mapping of channel to pin
		// see if we need to make a direction change, check array of [PWM pin][dir pin][dir]
		if( getCurrentDirection(channel) == 1) { // if dir 1, we are going what we define as 'forward' 
			if( motorPower < 0 ) { // and we want to go backward
				// reverse dir, send dir change to pin
				// default is 0 (LOW), if we changed the direction to reverse wheel rotation call the opposite dir change signal
				if(getDefaultDirection(channel) > 0) 
					Pins.getOutputPin(dirPinIndex).high();
				else 
					Pins.getOutputPin(dirPinIndex).low();
				setCurrentDirection(channel, 0); // set new direction value
				motorPower = -motorPower; //setMotorSpeed(channel,-motorPower); // absolute val
			}
		} else { // dir is 0
			if( motorPower > 0 ) { // we are going 'backward' as defined by our initial default direction and we want 'forward'
				// reverse, send dir change to pin
				/// default is 0 (HIGH), if we changed the direction to reverse wheel rotation call the opposite dir change signal
				if(getDefaultDirection(channel) > 0)  
					Pins.getOutputPin(dirPinIndex).low();
				else
					Pins.getOutputPin(dirPinIndex).high();
				setCurrentDirection(channel, 1);
			} else { // backward with more backwardness
				// If less than 0 take absolute value, if zero dont play with sign
				if( motorPower < 0) motorPower = -motorPower; //setMotorSpeed(channel,-motorPower); // absolute val;
			}
		}
		// scale motor power from 0-1000 
		if( motorPower != 0 && motorPower < getMinMotorPower(channel))
				motorPower = getMinMotorPower(channel);
		if( motorPower > getMaxMotorPower() ) // cap it at max
				motorPower = getMaxMotorPower();
		// Scale motor power if necessary and save it in channel speed array with proper sign for later use
		if( getMotorPowerScale() != 0 )
				motorPower /= getMotorPowerScale();
		//
		// Reset encoders on new speed setting
		resetEncoders();
		// If we have a linked distance sensor. check range and possibly skip
		// If we are setting power 0, we are stopping anyway
		if(checkUltrasonicShutdown()) {
			// find the PWM pin and get the object we set up in M3 to write to power level
			// element 0 of motorDrive has index to PWM array
			// writing power 0 sets mode 0 and timer turnoff
			setMotorShutdown(); // sends commandEmergencyStop(1);
			return fault_flag;
		}
		fault_flag = 0;
		if(motorPower == 0)
			Pins.getOutputPin(pIndex).low();
		else
			Pins.getOutputPin(pIndex).high();
		return 0;
	}

	@Override
	public int commandEmergencyStop(int status) throws IOException {
		for(int j=1; j <= channels; j++) {
			int pindex = getMotorDigitalPin(j);
			if(pindex != 255) {
				Pins.getOutputPin(getMotorEnablePin(j)).low(); // enable off
				Pins.getOutputPin(pindex).low(); // drive off
			}
		}
		fault_flag = status;
		resetSpeeds();
		resetEncoders();
		return status;
	}

	@Override
	public String getDriverInfo(int ch) {
		if( getMotorDigitalPin(ch) == 255 ) {
			return String.format("HB-SWITCH UNINITIALIZED Channel %d%n",ch);
		}
		return String.format("HB-SWITCH Channel %d Pin:%s, Dir Pin:%s%n",ch, Pins.getPin(getMotorDigitalPin(ch)), Pins.getPin(getMotorEnablePin(ch)));	
	}

}
