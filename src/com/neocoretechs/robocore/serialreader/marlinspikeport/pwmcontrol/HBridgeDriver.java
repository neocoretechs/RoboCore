package com.neocoretechs.robocore.serialreader.marlinspikeport.pwmcontrol;

import java.io.IOException;

/**
 * Generic driver for a collection of H bridge driven brushed DC motor channels.
 * Structure:
 * 1) Top level, a master list of pins in either the PWM or digitals arrays. Slots to which physical pins are assigned.
 * these are ppwms and pdigitals here, which hold pointers to these top level arrays.
 * 2) In each class and subclass, a sublist of pins. Multidimensional arrays which hold the index to the top level pins.
 * these are indexed by channel. motorDrive here. All PWM has a pin, and a frequency to 1000000 Hz.
 * 3) A PWM level indexed by channel. The current power or level value of the PWM timer. Standardized to reflect the control applied.
 * for instance, if the PS3 controller delivers -1000 to 1000 thats the range. If the controller is using the button that has -1 off and a max of 1000
 * then the range is 0 to 2000 to reflect the fact that no reverse operation is going to be in effect for something like a LED or pump.
 * 4) Optionally, a duration and a minimum PWM level, or here a minimum motor power. The duration represents a maximum interval before the PWM timer
 * is automatically shut down. In this case, the hall effect interrupts that are serviced by an interrupt handler that detect wheel rotation.
 * The minimum PWM level is the bottom limit for the PWM value, or in this case the minimum motor power. It is indexed by channel and the value is
 * the range that comes from the main controller, before any processing into a timer value.
 * the top level abstract class AbstractMotorControl contains these values.
 * Created: 10/2/2016 1:42:24 PM
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2025
 *
 */
public class HBridgeDriver extends AbstractPWMMotorControl {
	public static boolean DEBUG = false;
	final static String MSG_MOTORCONTROL_5= "Emergency stop";
	int status_flag = 0;
	
	public HBridgeDriver(int maxPower) {
		super(maxPower);
	}
	
	public int getMotorPWMPin(int channel) { return motorDrive[channel-1][0]; }
	public int getMotorEnablePin(int channel) {return motorDrive[channel-1][1]; }
	int getPWMFrequency(int channel) {return motorDrive[channel-1][2]; }

	@Override
	public int queryFaultFlag() { return fault_flag; }
	
	@Override
    public int queryStatusFlag() { return status_flag; }
	
	@Override
	public void resetMaxMotorPower() {
		MAXMOTORPOWER = 1000;	
	}
	
	@Override
	public int commandMotorPower(int channel, int motorPower) throws IOException {
		// check shutdown override
		if( MOTORSHUTDOWN )
			return 0;
		int pwmIndex = motorDrive[channel-1][0]; // index to PWM array
		int dirPinIndex = motorDrive[channel-1][1]; // index to dir pin array

		// get mapping of channel to pin
		// see if we need to make a direction change, check array of [PWM pin][dir pin][dir]
		if( getCurrentDirection(channel) == 1) { // if dir 1, we are going what we define as 'forward' 
			if( motorPower < 0 ) { // and we want to go backward
				// reverse dir, send dir change to pin
				// default is 0 (LOW), if we changed the direction to reverse wheel rotation call the opposite dir change signal
				if(getDefaultDirection(channel) > 0) 
					pdigitals[dirPinIndex] = 1;//.high();
				else 
					pdigitals[dirPinIndex] = 0;//.low();
				setCurrentDirection(channel, pdigitals[dirPinIndex]); // set new direction value
				motorPower = -motorPower; //setMotorSpeed(channel,-motorPower); // absolute val
			}
		} else { // dir is 0
			if( motorPower > 0 ) { // we are going 'backward' as defined by our initial default direction and we want 'forward'
				// reverse, send dir change to pin
				/// default is 0 (HIGH), if we changed the direction to reverse wheel rotation call the opposite dir change signal
				if(getDefaultDirection(channel) > 0)  
					pdigitals[dirPinIndex] = 0;//.low();
				else
					pdigitals[dirPinIndex] = 1;//.high();
				setCurrentDirection(channel, pdigitals[dirPinIndex]);
			} else { // backward with more backwardness
				// If less than 0 take absolute value, if zero dont play with sign
				if( motorPower < 0) motorPower = -motorPower; //setMotorSpeed(channel,-motorPower); // absolute val;
			}
		}

		// scale motor power
		if( motorPower != 0 && motorPower < getMinMotorPower(channel))
				motorPower = getMinMotorPower(channel);
		if( motorPower > getMaxMotorPower() ) // cap it at max
				motorPower = getMaxMotorPower();
		// Scale motor power if necessary and save it in channel speed array with proper sign for later use
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
		setMotorSpeed(channel, motorPower);
		ppwms[pwmIndex].freq(motorPower);
		ppwms[pwmIndex].duty(motorPower/2); //50% duty
		enable(channel);
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
		}
		fault_flag = status;
		resetSpeeds();
		resetEncoders();
		return status;
	}

	@Override
	public int isConnected() {
		return 1;
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
	public String getDriverInfo(int ch) {
		if( motorDrive[ch-1][0] == 255 ) {
			return String.format("HB-PWM UNINITIALIZED Channel %d%n",ch);
		}
		return String.format("HB-PWM Channel %d Pin:%d, Dir Pin:%d%n",ch, ppwms[motorDrive[ch-1][0]].pin, pdigitals[motorDrive[ch-1][0]]);	
	}

	@Override
	public void setInterruptServiceHandler(int intPin) {
		if(DEBUG)
			System.out.printf("%s.setInterrupterviceHandler pin %d%n", this.getClass().getName(),intPin);
		for(int j=0; j < channels; j++) {
			int pindex = motorDrive[j][0];
			if(pindex != 255 && intPin != 0 && intPin == ppwms[pindex].pin) {
				wheelEncoderService[j].setInterruptServiceHandler(ppwms[pindex]);
				if(DEBUG)
					System.out.printf("%s.setInterrupterviceHandler pin %d pindex %d channel %d encoder service %s pwm %s%n", this.getClass().getName(),intPin,pindex,j,wheelEncoderService[j],ppwms[pindex]);
				break;
			}
		}
	}

}
