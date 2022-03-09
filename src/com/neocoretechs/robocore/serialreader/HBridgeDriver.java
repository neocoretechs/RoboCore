package com.neocoretechs.robocore.serialreader;

import java.io.IOException;

import com.pi4j.io.gpio.GpioPinDigitalOutput;

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
 * @author groff
 *
 */
public class HBridgeDriver extends AbstractPWMMotorControl {
	PWM[] ppwms = new PWM[10];
	GpioPinDigitalOutput[] pdigitals = new GpioPinDigitalOutput[10];
	// 10 possible drive wheels, index is by channel-1. 
	// motorDrive[channel] {{PWM array index],[dir pin],[timer freq}}
	// PWM params array by channel:
	// 0-pin index to pwm array(default 255)
	// 1-direction pin
	// 2-timer frequency Hz
	int[][] motorDrive=new int[][]{{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000}};
	int status_flag = 0;
	public HBridgeDriver(int maxPower) {
		super(maxPower);
	}
	void setMotors(PWM[] pwm) { ppwms = pwm; }
	int getMotorPWMPin(int channel) { return motorDrive[channel-1][0]; }
	int getMotorEnablePin(int channel) {return motorDrive[channel-1][1]; }
	/**
	* Add a new PWM instance to this motor controller.
	* @param channel - the controller channel from 1 to 10, each successively higher channel will reset maximum channel count. Each channel is an axle/motor.
	* @param pin_number - the index in the PWM array defined in 'setMotors', this is the next blank slot available
	* @param dir_pin - the direction pin for this channel
	* @param dir_default - the default direction the motor starts in
	* @param timer_freq - timer frequency 0 - 1000000 Hz
	*/ 
	public void createPWM(int channel, int pin_number, int dir_pin, int dir_default, int timer_freq) {
		// Attempt to assign PWM pin
		if( getChannels() < channel ) setChannels(channel);
		GpioPinDigitalOutput opin =  Pins.assignPin(dir_pin);
		int dirpin;
		for(dirpin = 0;dirpin < 10; dirpin++) {
			if(pdigitals[dirpin] == null) {
				pdigitals[dirpin] = opin;
				break;
			}
		}
		int pindex;
		for(pindex = 0; pindex < 10; pindex++) {
			if( ppwms[pindex] == null ) {
					break;
			}
		}
		if( ppwms[pindex] != null  )
				return;
		currentDirection[channel-1] = dir_default;
		defaultDirection[channel-1] = dir_default;
				
		motorDrive[channel-1][0] = pindex;
		motorDrive[channel-1][1] = dir_pin;
		motorDrive[channel-1][2] = timer_freq;
		PWM ppin = new PWM(pin_number);
		ppwms[pindex] = ppin;
		ppwms[pindex].init(pin_number);
					
	}
	public void getDriverInfo(int ch, String outStr) {
		
	}
	@Override
	public int queryFaultFlag() { return fault_flag; }
	
	@Override
    public int queryStatusFlag() { return status_flag; }
	
	@Override
	public void resetMaxMotorPower() {
		MAXMOTORPOWER = 1000;	
	}

	void setDirectionPins(GpioPinDigitalOutput[] dpins) {
		pdigitals = dpins;
	}
	
	@Override
	public int commandMotorPower(int ch, int p) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public int commandEmergencyStop(int status) throws IOException {
		for(int j=0; j < 10; j++) {
			int pindex = motorDrive[j][0];
			if(pindex != 255) {
				//ppwms[pindex].init(ppwms[pindex].pin);
				ppwms[pindex].pwmOff();
			}
		}
		fault_flag = 16;
		resetSpeeds();
		resetEncoders();
		return status;
	}

	@Override
	public int isConnected() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public void getDriverInfo(int ch, char outStr) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public boolean checkUltrasonicShutdown() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean checkEncoderShutdown() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void createEncoder(int channel, int encode_pin) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public int getEncoderCount(int ch) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public void resetSpeeds() {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void resetEncoders() {
		// TODO Auto-generated method stub
		
	}

}
