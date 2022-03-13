package com.neocoretechs.robocore.serialreader.marlinspikeport.pwmcontrol;

import java.io.IOException;

/**
* Abstract class to facilitate control of multi channel or single channel PWM based devices that are not motors.<p/>
* Used to drive things like LED arrays and pumps that need variable speed controls without accompanying motor functionality
* like ultrasonic shutdown safety or hall effect counters for odometry or dead man switching. <p/>Provides the
* functionality of enable pin, etc to use H bridge driver or MOSFET circuit to drive things other than motors.<p/>
* IMPORTANT: Channels are referenced by the M and G code as a value from 1-10 but the indexes are from 0-9 in the arrays
* of channels, therefore, always subtract 1 when going from methods that take values from the processing loop, such as:<p/>
* commandMotorPower<br/>
* getDriverInfo<br/>
*<p/>
* Structure:<p/>
* 1) Top level, a master list of pins in either the PWM or digitals arrays. Slots to which physical pins are assigned.
* these are ppwms and pdigitals here, which hold pointers to these top level arrays.<p/>
* 2) In each class and subclass, a sublist of pins. Multidimensional arrays which hold the index to the top level pins.
* these are indexed by channel. PWMDrive here. All PWM has a pin and a frequency.<p/>
* 3) A PWM level indexed by channel. The current power or level value of the PWM timer. Standardized to reflect the control applied.
* for instance, if the PS3 controller delivers -1000 to 1000 thats the range. If the controller is using the button that has -1 off and a max of 1000
* then the range is 0 to 2000 to reflect the fact that no reverse operation is going to be in effect for something like a LED or pump.<p/>
* 4) Optionally, a PWM duration and a minimum PWM level. The PWM duration represents a maximum time interval before the PWM timer 
* is automatically shut down. The minimum PWM level is the bottom limit for the PWM value. It is indexed by channel and the value is
* the range that comes from the main controller, before any processing into a timer value.<p/>
*
* channel - the controller channel from 1 to 10, each successively higher channel will reset maximum channel count. Each channel is a device on an
* Hbridge split into separate outputs, or a motor driver attached to an LED array, etc.<p/>
* pin_number - the index in the PWM array defined in 'setPWM', this is the next blank slot available<br/>
* enable_pin - the enable pin for this channel. Assumed that low is disabled, high is enable.<br/>
* frequency - PWM frequency from 1 - 1000000<br/>
* Originally Created: 10/29/2020 9:54:23 AM
* @author Jonathan Groff Copyright (C) NeoCoreTechs 2022
*/
public abstract class AbstractPWMControl {
	protected int channels = 10;
	protected int status_flag = 0;
	// 10 possible drive channels, index is by channel-1.
	// pwmDrive[channel] [[PWM array index][dir pin][pwm freq]
	// PWM params array by channel:
	// 0-pin index to pwm array(default 255)
	// 1-enable pin
	// 2-pwm frequency
	protected int[][] pwmDrive=new int[][]{{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000}};
	protected int[] maxPWMDuration = new int[]{-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}; // number of milliseconds operation before auto shutdown
	// 10 channels of last PWM value
	protected int[] pwmLevel = new int[]{0,0,0,0,0,0,0,0,0,0};
	protected int[] minPWMLevel = new int[]{0,0,0,0,0,0,0,0,0,0}; // Offset to add to G6, use with care, meant to compensate for electrical differences
	protected int PWMPOWERSCALE = 0; // scale, divisor for PWM level to reduce 0-1000 scale if non zero
	protected int PWMSHUTDOWN = 0; // Override of PWM controls, puts it in irons
	protected int fault_flag = 0;
	//functions
	public void setDuration(int ch, int durx) { maxPWMDuration[ch-1] = durx; }
	public void setPWMLevel(int ch, int lvl) { pwmLevel[ch-1] = lvl; }
	public void setMinPWMLevel(int ch, int mpow) { minPWMLevel[ch-1] = mpow;}
	public abstract int commandPWMLevel(int ch, int p);
	public abstract int commandEmergencyStop(int status) throws IOException;
	protected abstract int isConnected();
	public abstract String getDriverInfo(int ch);
	protected abstract int queryFaultFlag();
	protected abstract int queryStatusFlag();
	public abstract void setMaxPWMLevel(int p);
	public  int getMaxPWMDuration(int ch) { return maxPWMDuration[ch-1]; }
	public int getMinPWMLevel(int ch) { return minPWMLevel[ch-1] ; }
	protected int getPWMLevel(int ch) { return pwmLevel[ch-1]; }
	protected void setChannels(int ch) { channels = ch; }
	public int getChannels() { return channels; }
	protected abstract void resetLevels();
	public void setPWMShutdown() throws IOException { commandEmergencyStop(1); PWMSHUTDOWN = 1;}
	public void setPWMRun() throws IOException { commandEmergencyStop(0); PWMSHUTDOWN = 0;}
	protected int getPWMShutdown() { return PWMSHUTDOWN; }
	public void setPWMPowerScale(int p) { PWMPOWERSCALE = p; }
}
