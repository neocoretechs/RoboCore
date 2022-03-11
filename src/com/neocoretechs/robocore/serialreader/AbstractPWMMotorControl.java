package com.neocoretechs.robocore.serialreader;

import java.io.IOException;

import com.pi4j.io.gpio.GpioPinDigitalOutput;
/**
 * Abstract driver for a collection of H bridge driven brushed DC motor channels.
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
 * @author groff
 *
 */
public abstract class AbstractPWMMotorControl extends AbstractMotorControl {
	PWM[] ppwms = new PWM[channels];
	GpioPinDigitalOutput[] pdigitals = new GpioPinDigitalOutput[channels];
	// 10 possible drive wheels, index is by channel-1. 
	// motorDrive[channel] {{PWM array index],[dir pin],[timer freq}}
	// PWM params array by channel:
	// 0-pin index to pwm array(default 255)
	// 1-direction pin
	// 2-timer frequency Hz
	int[][] motorDrive=new int[][]{{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000}};
	public AbstractPWMMotorControl(int maxPower) {
		MAXMOTORPOWER = maxPower;
	}
	/**
	* Add a new PWM instance to this motor controller.
	* @param channel - the controller channel from 1 to 10, each successively higher channel will reset maximum channel count. Each channel is an axle/motor.
	* @param pin_number - the index in the PWM array defined in 'setMotors', this is the next blank slot available
	* @param dir_pin - the direction pin for this channel
	* @param dir_default - the default direction the motor starts in
	* @param timer_freq - timer frequency 0 - 1000000 Hz
	* @throws IOException 
	*/ 
	public void createPWM(int channel, int pin_number, int dir_pin, int dir_default, int timer_freq) throws IOException {
		// Attempt to assign PWM pin
		if( getChannels() < channel ) setChannels(channel);
		GpioPinDigitalOutput opin = Pins.assignPin(dir_pin);
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
		setCurrentDirection(channel, dir_default);
		setDefaultDirection(channel, dir_default);
		setMotorSpeed(channel, 0);	
		motorDrive[channel-1][0] = pindex;
		motorDrive[channel-1][1] = dirpin;
		motorDrive[channel-1][2] = timer_freq;
		PWM ppin = new PWM(pin_number);
		ppwms[pindex] = ppin;
		ppwms[pindex].init(pin_number, timer_freq);
					
	}
	public abstract void resetMaxMotorPower();
	
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
