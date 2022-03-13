package com.neocoretechs.robocore.serialreader;

import java.io.IOException;

import com.pi4j.io.gpio.GpioPinDigitalOutput;

/**
* VariablePWMDriver
* Driver to control a PWM device that is not a propulsion motor, such as LED or pump, and as such
* has no recognition of odometry or safety interlock or ultrasonic distance shutdown.<p/>
* IMPORTANT: Channels are referenced by the M and G code as a value from 1-10 but the indexes are from 0-9 in the arrays
* of channels, therefore, always subtract 1 when going from methods that take values from the processing loop, such as:<p/>
* commandMotorPower<br/>
* getDriverInfo<br/>
*
* Structure:<p/>
* 1) Top level, a master list of pins in either the PWM or digitals arrays. Slots to which physical pins are assigned.
* these are ppwms and pdigitals here, which hold pointers to these top level arrays.<p/>
* 2) In each class and subclass, a sublist of pins. Multidimensional arrays which hold the index to the top level pins.
* these are indexed by channel. PWMDrive here. All PWM has a pin and a frequency.<p/
* 3) A PWM level indexed by channel. The current power or level value of the PWM timer. Standardized to reflect the control applied.
* for instance, if the PS3 controller delivers -1000 to 1000 thats the range. If the controller is using the button that has -1 off and a max of 1000
* then the range is 0 to 2000 to reflect the fact that no reverse operation is going to be in effect for something like a LED or pump.<p/>
* 4) Optionally, a PWM duration and a minimum PWM level. The PWM duration represents a maximum time interval before the PWM timer
* is automatically shut down. The minimum PWM level is the bottom limit for the PWM value. It is indexed by channel and the value is
* the range that comes from the main controller, before any processing into a timer value.<p/>
* These arrays can be located in the top level abstract class {@link AbstractPWMControl}.
* Created: 10/29/2020 3:15:01 PM
* @author Jonathan N Groff Copyright (C) NeoCoreTechs 2020,2022
*
*/
public class VariablePWMDriver extends AbstractPWMControl {
	public static int MAXPWMLEVEL = 2000;
	PWM[] ppwms = new PWM[channels];
	GpioPinDigitalOutput[] pdigitals = new GpioPinDigitalOutput[channels];
	int getPWMLevelPin(int channel) { return pwmDrive[channel-1][0]; }
	int getPWMEnablePin(int channel) {return pwmDrive[channel-1][1]; }
		
	@Override
	protected int queryFaultFlag() { return fault_flag; }
		
	@Override
	protected int queryStatusFlag() { return status_flag; }
	
	/**
	* Add a new PWM instance to this controller.<p/>
	* channel - the controller channel from 1 to 10, each successively higher channel will reset maximum channel count. Each channel is a device on an 
	* Hbridge split into separate outputs, or a motor driver attached to an LED array, etc.<p/>
	* @param channel
	* @param pin_number - the index in the PWM array defined in 'setPWM', this is the next blank slot available
	* @param enable_pin - the enable pin for this channel. Assumed that low is disabled, high is enable.
	* @param dir_default - default 'direction' normally on or off
	* @param timer_freq - timer prescale default 1 = no prescale
	* @throws IOException 
	*/
	void createPWM(int channel, int pin_number, int enable_pin, int dir_default, int timer_freq) throws IOException {
		// Attempt to assign PWM pin
		if( getChannels() < channel ) setChannels(channel);
		GpioPinDigitalOutput opin = Pins.assignPin(enable_pin);
		int dirpin;
		for(dirpin = 0;dirpin < channels; dirpin++) {
			if(pdigitals[dirpin] == null) {
				pdigitals[dirpin] = opin;
				break;
			}
		}
		int pindex;
		for(pindex = 0; pindex < channels; pindex++) {
			if( ppwms[pindex] == null ) {
					break;
			}
		}
		pwmDrive[channel-1][0] = pindex;
		pwmDrive[channel-1][1] = dirpin;
		pwmDrive[channel-1][2] = timer_freq;
		PWM ppin = new PWM(pin_number);
		ppwms[pindex] = ppin;
		ppwms[pindex].init(pin_number, timer_freq);
	}	
	
	@Override
	protected int commandPWMLevel(int ch, int p) {
		// TODO Auto-generated method stub
	return 0;
	}

	@Override
	protected int commandEmergencyStop(int status) throws IOException {
		for(int j=0; j < 10; j++) {
			int pindex = pwmDrive[j][0];
			if(pindex != 255) {
					pdigitals[pindex].low();
			}
			pindex = pwmDrive[j][0];
			if(pindex != 255) {
				ppwms[pindex].pwmOff();
			}
		}
		fault_flag = 16;
		resetLevels();
		return status;
	}

	@Override
	protected int isConnected() {
			return 1;
	}

	@Override
	protected String getDriverInfo(int ch) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	protected void setMaxPWMLevel(int p) {
		MAXPWMLEVEL = p;
	}

	@Override
	protected void resetLevels() {
		// TODO Auto-generated method stub

	}

}
