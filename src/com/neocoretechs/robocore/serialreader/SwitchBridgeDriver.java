package com.neocoretechs.robocore.serialreader;

import java.io.IOException;

import com.pi4j.io.gpio.GpioPinDigital;
import com.pi4j.io.gpio.GpioPinDigitalOutput;
/**
*SwitchBridgeDriver:
* A motor driver that uses straight GPIO switching rather than PWM, for an on/off drive motor setup.<p/>
*
* Structure:<p/>
* 1) Top level, a master list of pins in the digitals array. Slots to which physical pins are assigned.
* these are pdigitals here, which hold pointers to these top level array.<p/>
* 2) In each class and subclass, a sublist of pins. Multidimensional array which holds the index to the top level pins.
* these are indexed by channel. motorDrive here.<p/>
* Since the M1 and M2 inputs are split, we add an additional motorDriveB multidimensional array for the second input.
* NOTE: The pin assignment for the IO pins is arranged in the index array one after the other, so the assignment for M1 is at pindex
* and the assignment for M2 is at pindex+1. This does NOT mean the pins have to be next to each other, its just that the indexes in
* the arrays are arranged one after the other for convenience.
* 3) A GPIO pin list indexed by channel. The level value is either a + or - value for on or off.<p/>
* 4) Optionally, a duration. The duration represents a maximum interval before the GPIO pin
* is automatically shut down. In this case, the hall effect interrupts that are serviced by an interrupt handler that detect wheel rotation.
* the top level abstract class AbstractMotorControl contains these values.<p/>
* Created: 10/20/2020 1:39:44 PM
* @author groff
*
*/
public class SwitchBridgeDriver extends AbstractMotorControl {
	GpioPinDigitalOutput[] pdigitals = new GpioPinDigitalOutput[channels*4];
	// 5 possible drive wheels, index is by channel-1.
	// motorDrive[channel] [[Digitals array index][dir pin]
	// PWM params array by channel:
	// 0-pin index to Digital pins array(default 255)
	// 1-direction pin
	int[][] motorDrive= new int[][]{{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0}};
	int[][] motorDriveB= new int[][]{{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0}};
	int status_flag = 0;
	
	int getMotorDigitalPin(int channel) { return motorDrive[channel-1][0]; }
	
	int getMotorDigitalPinB(int channel) { return motorDriveB[channel-1][0]; }

	int getMotorEnablePin(int channel) {return motorDrive[channel-1][1]; }
	
	int getMotorEnablePinB(int channel) {return motorDriveB[channel-1][1]; }
	/**
	 * Add a new Digital pin instance to this motor controller.
	 * @param channel the controller channel from 1 to channels
	 * @param pin_number the index in the GPIO pin array defined in 'setMotors' for M1, this is the next blank slot available
	 * @param pin_numberB the index in the GPIO array defined in 'setMotors' for M2
	 * @param dir_pin the enable pin for this channel
	 * @param dir_pinB enable for B
	 * @param dir_default the default direction the motor starts in
	 */
	void createDigital(int channel, int pin_number, int pin_numberB, int dir_pin, int dir_pinB, int dir_default) {
		int i;
		for(i = 0; i < channels*4; i++) {
			if(pdigitals[i] == null)
				break;
		}
		pdigitals[i] = Pins.assignPin(pin_number);
		motorDrive[channel-1][0] = i;
		for(i = 0; i < channels*4; i++) {
			if(pdigitals[i] == null)
				break;
		}
		pdigitals[i] = Pins.assignPin(pin_numberB);
		motorDriveB[channel-1][0] = i;
		for(i = 0; i < channels*4; i++) {
			if(pdigitals[i] == null)
				break;
		}	
		pdigitals[i] = Pins.assignPin(dir_pin);
		motorDrive[channel-1][1] = i;
		for(i = 0; i < channels*4; i++) {
			if(pdigitals[i] == null)
				break;
		}
		pdigitals[i] = Pins.assignPin(dir_pinB);
		motorDriveB[channel-1][1] = i;
				
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
		int gioIndex = motorDrive[channel][0]; // index to gpio array
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
		pdigitals[gioIndex].high();
		// now do B, if dir pin is same, values should match, otherwise pin may be written
		gioIndex = motorDriveB[channel][0]; // index to gpio array
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
		pdigitals[gioIndex].high();
		return 0;
	}

	@Override
	public int commandEmergencyStop(int status) throws IOException {
		for(int j=0; j < channels; j++) {
			int pindex = motorDrive[j][0];
			if(pindex != 255) {
				pdigitals[pindex].low();
			}
			pindex = motorDriveB[j][0];
			if(pindex != 255) {
					pdigitals[pindex].low();
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
		if( motorDrive[ch-1][0] == 255 ) {
			return String.format("SwitchBridge UNINITIALIZED Channel %d%n",ch);
		}
		return String.format("SwitchBridge Channel %d PinA:%d, PinB:%d Dir PinA:%d Dir PinB:%d%n",ch, motorDrive[ch-1][0], motorDriveB[ch-1][0], motorDrive[ch-1][1], motorDrive[ch-1][1]);	
	}
	
	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder();
		for(int i = 0; i < channels; i ++) {
			if(motorDrive[i+1][0] != 255)
				sb.append(getDriverInfo(i+1));
		}
		return sb.toString();
	}

}