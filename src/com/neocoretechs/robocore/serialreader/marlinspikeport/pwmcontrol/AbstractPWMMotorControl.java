package com.neocoretechs.robocore.serialreader.marlinspikeport.pwmcontrol;

import java.io.IOException;

import com.neocoretechs.robocore.propulsion.PWM;
import com.neocoretechs.robocore.serialreader.marlinspikeport.Pins;
import com.neocoretechs.robocore.serialreader.marlinspikeport.control.AbstractMotorControl;

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
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2025
 *
 */
public abstract class AbstractPWMMotorControl extends AbstractMotorControl {
	private static boolean DEBUG = true;
	protected PWM[] ppwms = new PWM[channels];
	// 10 possible drive wheels, index is by channel-1. 
	// motorDrive[channel] {{PWM array index],[dir pin],[timer freq}}
	// PWM params array by channel:
	// 0-pin index to pwm array(default 255)
	// 1-direction pin
	// 2-timer frequency Hz
	protected int[][] motorDrive=new int[][]{
		{255,0,50000,25000},
		{255,0,50000,25000},
		{255,0,50000,25000},
		{255,0,50000,25000},
		{255,0,50000,25000},
		{255,0,50000,25000},
		{255,0,50000,25000},
		{255,0,50000,25000},
		{255,0,50000,25000},
		{255,0,50000,25000}};
		
	public AbstractPWMMotorControl(int maxPower) {
		MAXMOTORPOWER = maxPower;
	}
	/**
	* Add a new PWM instance to this motor controller.
	* @param channel - the controller channel from 1 to 10, each successively higher channel will reset maximum channel count. Each channel is an axle/motor.
	* @param pin_number - the index in the PWM array defined in 'setMotors', this is the next blank slot available
	* @param dir_pin - the direction pin for this channel
	* @param dir_default - the default direction the motor starts in
	* @param freq - timer frequency 0 - 1000000 Hz
	* @param duty - duty cycle must be < freq
	* @throws IOException 
	*/ 
	public void createPWM(int channel, int pin_number, int dir_pin, int dir_default, int freq, int duty) throws IOException {
		// Attempt to assign PWM pin
		if( channel < 0 || channel >= getChannels()  ) {
				throw new IOException(String.format("%s channel NOT set to:%d since getChannels() reported %d%n",this.getClass().getName(), channel, getChannels()));
		}	
		Pins.assignPin(dir_pin);
		if(DEBUG)
			System.out.printf("%s direction pin %d assigned%n",this.getClass().getName(), dir_pin);
		
		int pindex;
		for(pindex = 0; pindex < channels; pindex++) {
			if( ppwms[pindex] == null ) {
					break;
			}
		}
		if(pindex == channels)
			System.out.println("PWM pin slot exceeded available channels:"+pindex);
		if(DEBUG)
			System.out.printf("%s PWM pin slot set to:%d%n",this.getClass().getName(), pindex);
		
		setCurrentDirection(channel, dir_default);
		setDefaultDirection(channel, dir_default);
		setMotorSpeed(channel, 0);	
		motorDrive[channel-1][0] = pindex;
		motorDrive[channel-1][1] = dir_pin;
		motorDrive[channel-1][2] = freq;
		motorDrive[channel-1][3] = duty;
		PWM ppin = new PWM(pin_number);
		if(DEBUG)
			System.out.printf("%s PWM instance set to:%s%n",this.getClass().getName(), ppin);
		ppwms[pindex] = ppin;
		if(DEBUG)
			System.out.printf("%s PWM instance initialized to:%s%n",this.getClass().getName(), ppwms[pindex]);				
	}
	
	public abstract void resetMaxMotorPower();
	
	@Override
	public void enable(int ch) throws IOException {
		ppwms[motorDrive[ch-1][0]].enable(true);
	}
	
	@Override
	public void disable(int ch) throws IOException {
		ppwms[motorDrive[ch-1][0]].enable(false);
	}
	
	@Override
	public String toString() {
		StringBuilder sb = new StringBuilder();
		for(int i = 1; i < channels; i++) {
			if(motorDrive[i][0] != 255)
				sb.append(getDriverInfo(i));
		}
		return sb.toString();
	}
}
