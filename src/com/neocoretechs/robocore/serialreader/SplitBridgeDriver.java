package com.neocoretechs.robocore.serialreader;

import java.io.IOException;

import com.pi4j.io.gpio.GpioPinDigitalOutput;

/**
* SplitBridgeDriver<p/>
* This is a type of H bridge with 2 separate inputs for M1 and M2, such that is may function as several types of drivers.<p/>
* An H-bridge with an A and B side that are controllable as H or 2 half bridges.<p/>
* H-bridge, half bridge, or ganged half bridge.<br/>
* As a regular H-bridge, 2 PWM inputs are necessary. <p/>
* Both half-bridges may operate independently or they can be ganged together in parallel to
* support approximately double the current of the H-bridge or single half-bridge configuration.<p/>
* PA PWM Fwd <br/>
* PB PWM Rev <br/>
* EA Enable Mot1 and Mot2 (jumper)<br/>
* ----- Presumably a forward motor and a reverse motor (or circuit for forward, and one for reverse) <p/>
* PA PWM Motor 1 <br/>
* PB PWM Motor 2 <br/>
* EA Enable Mot 1 <br/>
* EB Enable Mot 2 <br/>
* ----- <br/>
* PA PWM M1 <br/>
* PB Tie to PA <br/>
* EA Enable <br/>
*<p/>
* Structure:<p/>
* 1) Top level, a master list of pins in either the PWM or digitals arrays. Slots to which physical pins are assigned.
* these are ppwms and pdigitals here, which hold pointers to these top level arrays.<p/>
* 2) In each class and subclass, a sublist of pins. Multidimensional arrays which hold the index to the top level pins.
* these are indexed by channel. motorDrive here. All PWM has a pin, a prescale, and a resolution. We standardize the resolution to 8 bits typically.
* Since the M1 and M2 inputs are split, we add an additional motorDriveB multidimensional array for the second input.<p/>
* NOTE: The pin assignment for the timer pins is arranged in the index array one after the other, so the assignment for M1 is at pindex
* and the assignment for M2 is at pindex+1. This does NOT mean the pins have to be next to each other, its just that the indexes in
* the arrays are arranged one after the other for convenience.<p/>
* 3) A PWM level indexed by channel. The current power or level value of the PWM timer. Standardized to reflect the control applied.
* for instance, if the PS3 controller delivers -1000 to 1000 thats the range. If the controller is using the button that has -1 off and a max of 1000
* then the range is 0 to 2000 to reflect the fact that no reverse operation is going to be in effect for something like a LED or pump.<p/>
* 4) Optionally, a duration and a minimum PWM level, or here a minimum motor power. The duration represents a maximum interval before the PWM timer
* is automatically shut down. In this case, the hall effect interrupts that are serviced by an interrupt handler that detect wheel rotation.
* The minimum PWM level is the bottom limit for the PWM value, or in this case the minimum motor power. It is indexed by channel and the value is
* the range that comes from the main controller, before any processing into a timer value.<p/>
* The top level abstract class AbstractMotorControl contains these values.<p/>
* Created: 10/16/2020 12:40:41 PM
* @author Jonathan Groff Copyright (C) NeoCoreTechs 2022
*
*/
public class SplitBridgeDriver extends HBridgeDriver {
	// frequency value held in superclass
	// channels 1-10 no 0, pad, always sub 1
	private int[][] motorDriveB= new int[][]{{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0}};
	public SplitBridgeDriver(int maxPower) {
		super(maxPower);
	}
	int getMotorPWMPinB(int channel) { return motorDriveB[channel-1][0]; }
	/**
	 * Add a new PWM instance to this motor controller.
	 * @param channel - the controller channel from 1 to 10
	 * @param pin_numberA - the index in the PWM array defined in 'setMotors' for M1, this is the next blank slot available
	 * @param pin_numberB - the index in the PWM array defined in 'setMotors' for M2
	 * @param dir_pinA - the enable pin for channel A
	 * @param dir_pinB - the enable pin for channel B (may be same as A)
	 * @param dir_default - the default direction the motor starts in
	 * @param timer_freq - timer resolution in bits - default 8
	 * @throws IOException 
	 */ 
	public void createPWM(int channel, int pin_numberA, int pin_numberB, int dir_pinA, int dir_pinB, int dir_default, int timer_freq) throws IOException {
		super.createPWM(channel, pin_numberA, dir_pinA, dir_default, timer_freq);
		// Attempt to assign PWM pin
		int pindex;
		for(pindex = 0; pindex < channels; pindex++) {
			if( ppwms[pindex] == null ) {
					break;
			}
		}
		setCurrentDirection(channel, dir_default);
		setDefaultDirection(channel, dir_default);
		setMotorSpeed(channel, 0);	
		motorDriveB[channel-1][0] = pindex;
		motorDriveB[channel-1][1] = dir_pinB;
		motorDriveB[channel-1][2] = timer_freq;
		PWM ppin = new PWM(pin_numberB);
		ppwms[pindex] = ppin;
		ppwms[pindex].init(pin_numberB, timer_freq);				
	}
	
	@Override
	public int commandMotorPower(int channel, int motorPower) throws IOException {
		// check shutdown override
		if( MOTORSHUTDOWN )
			return 0;
		int pwmIndex = motorDrive[channel][0]; // index to PWM array
		int dirPinIndex = motorDrive[channel][1]; // index to dir pin array
		//int freq = motorDrive[channel][2]; // value of freq, no index;
		// get mapping of channel to pin
		// see if we need to make a direction change, check array of [PWM pin][dir pin][dir]
		if( getCurrentDirection(channel) == 1) { // if dir 1, we are going what we define as 'forward' 
			if( motorPower < 0 ) { // and we want to go backward
				// reverse dir, send dir change to pin
				// default is 0 (LOW), if we changed the direction to reverse wheel rotation call the opposite dir change signal
				if(getDefaultDirection(channel) > 0) 
					pdigitals[dirPinIndex].high();
				else 
					pdigitals[dirPinIndex].low();
				setCurrentDirection(channel, 0); // set new direction value
				motorPower = -motorPower; //setMotorSpeed(channel,-motorPower); // absolute val
			}
		} else { // dir is 0
			if( motorPower > 0 ) { // we are going 'backward' as defined by our initial default direction and we want 'forward'
				// reverse, send dir change to pin
				/// default is 0 (HIGH), if we changed the direction to reverse wheel rotation call the opposite dir change signal
				if(getDefaultDirection(channel) > 0)  
					pdigitals[dirPinIndex].low();
				else
					pdigitals[dirPinIndex].high();
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
		if( !checkUltrasonicShutdown()) {
			// find the PWM pin and get the object we set up in M3 to write to power level
			// element 0 of motorDrive has index to PWM array
			// writing power 0 sets mode 0 and timer turnoff
			setMotorShutdown(); // sends commandEmergencyStop(1);
			return fault_flag;
		}
		fault_flag = 0;
		//ppwms[pwmIndex].freq(freq);
		ppwms[pwmIndex].pwmWrite(motorPower);
		// now do B, if dir pin is same, values should match, otherwise pin may be written
		pwmIndex = motorDriveB[channel][0]; // index to PWM array
		dirPinIndex = motorDriveB[channel][1]; // index to dir pin array
		//int freq = motorDrive[channel][2]; // value of freq, no index;
		// get mapping of channel to pin
		// see if we need to make a direction change, check array of [PWM pin][dir pin][dir]
		if( getCurrentDirection(channel) == 1) { // if dir 1, we are going what we define as 'forward' 
			if( motorPower < 0 ) { // and we want to go backward
				// reverse dir, send dir change to pin
				// default is 0 (LOW), if we changed the direction to reverse wheel rotation call the opposite dir change signal
				if(getDefaultDirection(channel) > 0) 
					pdigitals[dirPinIndex].high();
				else 
					pdigitals[dirPinIndex].low();
				setCurrentDirection(channel, 0); // set new direction value
				motorPower = -motorPower; //setMotorSpeed(channel,-motorPower); // absolute val
			}
		} else { // dir is 0
			if( motorPower > 0 ) { // we are going 'backward' as defined by our initial default direction and we want 'forward'
				// reverse, send dir change to pin
				/// default is 0 (HIGH), if we changed the direction to reverse wheel rotation call the opposite dir change signal
				if(getDefaultDirection(channel) > 0)  
					pdigitals[dirPinIndex].low();
				else
					pdigitals[dirPinIndex].high();
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
		if( !checkUltrasonicShutdown()) {
			// find the PWM pin and get the object we set up in M3 to write to power level
			// element 0 of motorDrive has index to PWM array
			// writing power 0 sets mode 0 and timer turnoff
			setMotorShutdown(); // sends commandEmergencyStop(1);
			return fault_flag;
		}
		fault_flag = 0;
		//ppwms[pwmIndex].freq(freq);
		ppwms[pwmIndex].pwmWrite(motorPower);
		return 0;
	}

	@Override
	public int commandEmergencyStop(int status) throws IOException {
		for(int j=0; j < channels; j++) {
			int pindex = motorDrive[j][0];
			if(pindex != 255) {
				//ppwms[pindex].init(ppwms[pindex].pin);
				ppwms[pindex].pwmOff();
			}
			pindex = motorDriveB[j][0];
			if(pindex != 255) {
				ppwms[pindex].pwmOff();
			}
		}
		fault_flag = 16;
		resetSpeeds();
		resetEncoders();
		return status;
	}
	
	@Override
	public String getDriverInfo(int ch) {
		if( motorDrive[ch-1][0] == 255 ) {
			return String.format("SplitBridge-PWM UNINITIALIZED Channel %d%n",ch);
		}
		return String.format("SplitBridge-PWM Channel %d PinA:%d, PinB:%d Dir Pin:%s%n",ch, ppwms[motorDrive[ch-1][0]].pin, ppwms[motorDriveB[ch-1][0]].pin, pdigitals[motorDrive[ch-1][0]].getPin());	
	}
	
	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder();
		for(int i = 0; i < channels; i ++) {
			if(motorDriveB[i+1][0] != 255)
				sb.append(getDriverInfo(i+1));
		}
		return sb.toString();
	}

}
