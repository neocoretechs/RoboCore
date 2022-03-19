package com.neocoretechs.robocore.serialreader.marlinspikeport.pwmcontrol;

import java.io.IOException;

import com.neocoretechs.robocore.serialreader.marlinspikeport.PWM;
import com.neocoretechs.robocore.serialreader.marlinspikeport.Pins;
import com.pi4j.io.gpio.GpioPinDigitalOutput;
import com.pi4j.io.gpio.PinMode;

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
	public int getPWMLevelPin(int channel) { return pwmDrive[channel-1][0]; }
	public int getPWMEnablePin(int channel) {return pwmDrive[channel-1][1]; }
		
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
	public void createPWM(int channel, int pin_number, int enable_pin, int dir_default, int timer_freq) throws IOException {
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
	/**
	 * Command the driver power level. Manage enable pin. If necessary limit min and max power and
	 * scale to the SCALE if > 0. After calculation and saved values in the 0-2000 range scale it to 0-255 for 8 bit PWM.
	 * Instead of our -1000 to 1000 range from stick we change it to 0-2000 by adding 1000 since no reverse is relevant.
	 * In the case of Ps3 controller the axis will have the zero point of the stick as halfway, and full stick back
	 * as 0, in the case of a trigger, the output is from -1 to 1 with no output being -1, so it will function as desired and expected.
	 * Each channel is a PWM driven device.
	 * @throws IOException 
	 */
	public int commandPWMLevel(int pwmChannel, int pwmPower) throws IOException {
		// check shutdown override
		if( PWMSHUTDOWN )
			return 0;
		boolean foundPin = false;
		pwmPower += 1000;
		pwmLevel[pwmChannel-1] = pwmPower;
		// get mapping of channel to pin
		int ePin = getPWMEnablePin(pwmChannel);
		if(pdigitals[ePin] != null  ) {
				pdigitals[ePin].setMode(PinMode.DIGITAL_OUTPUT);
				pdigitals[ePin].high();
				foundPin = true;
		}
		if(!foundPin) {
			return commandEmergencyStop(7);
		}                                                                                                                                                     
		// scale motor power from 0-2000 to our 0-255 8 bit timer val
		pwmPower /= 8;
		//
		if( pwmPower != 0 && pwmPower < minPWMLevel[pwmChannel-1])
			pwmPower = minPWMLevel[pwmChannel-1];
		if( pwmPower > MAXPWMLEVEL ) // cap it at max
			pwmPower = MAXPWMLEVEL;
		// Scale motor power if necessary and save it in channel speed array with proper sign for later use
		if( PWMPOWERSCALE != 0 )
			pwmPower /= PWMPOWERSCALE;
		//
		// find the PWM pin and get the object we set up in M3 to write to power level
		int timer_freq = pwmDrive[pwmChannel-1][3]; // timer resolution in bits from M3
		// element 0 of motorDrive has index to PWM array
		int pindex = pwmDrive[pwmChannel-1][0];
		// writing power 0 sets mode 0 and timer turnoff
		ppwms[pindex].init(ppwms[pindex].pin, timer_freq);
		//ppwms[pindex]->attachInterrupt(motorDurationService[motorChannel-1]);// last param TRUE indicates an overflow interrupt
		ppwms[pindex].pwmWrite(pwmPower);
		fault_flag = 0;
		return 0;
	}

	@Override
	public int commandEmergencyStop(int status) throws IOException {
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
	public String getDriverInfo(int ch) {
		if( pwmDrive[ch-1][0] == 255 ) {
			return String.format("Variable-PWM UNINITIALIZED Channel %d%n",ch);
		}
		return String.format("Variable-PWM Channel %d Pin:%d, Dir Pin:%s%n",ch, ppwms[pwmDrive[ch-1][0]].pin, pdigitals[pwmDrive[ch-1][0]].getPin());	
	}

	@Override
	public void setMaxPWMLevel(int p) {
		MAXPWMLEVEL = p;
	}

	@Override
	protected void resetLevels() {
		int pindex;
		for(pindex = 0; pindex < channels; pindex++) {
			if( ppwms[pindex] != null ) {
					try {
						ppwms[pindex].pwmWrite(0);
					} catch (IOException e) {
						e.printStackTrace();
					}
			}
		}
	}

}
