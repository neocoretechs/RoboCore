package com.neocoretechs.robocore.serialreader;
/**
* This is a driver for an H bridge that has separate M1 and M2 inputs.<p/>
* Structure:<p/>
* 1) Top level, a master list of pins in either the PWM or digitals arrays. Slots to which physical pins are assigned.
* these are ppwms and pdigitals here, which hold pointers to these top level arrays.<p/>
* 2) In each class and subclass, a sublist of pins. Multidimensional arrays which hold the index to the top level pins.
* these are indexed by channel. motorDrive here. All PWM has a pin, and a frequency.
* Since the M1 and M2 inputs are split, we add an additional motorDriveB multidimensional array for the second input.
* NOTE: The pin assignment for the timer pins is arranged in the index array one after the other, so the assignment for M1 is at pindex
* and the assignment for M2 is at pindex+1. This does NOT mean the pins have to be next to each other, its just that the indexes in
* the arrays are arranged one after the other for convenience.<p/>
* 3) A PWM level indexed by channel. The current power or level value of the PWM timer. Standardized to reflect the control applied.
* for instance, if the PS3 controller delivers -1000 to 1000 thats the range. If the controller is using the button that has -1 off and a max of 1000
* then the range is 0 to 2000 to reflect the fact that no reverse operation is going to be in effect for something like a LED or pump.<p/>
* 4) Optionally, a duration and a minimum PWM level, or here a minimum motor power. The duration represents a maximum interval before the PWM timer
* is automatically shut down. In this case, the hall effect interrupts that are serviced by an interrupt handler that detect wheel rotation.
* The minimum PWM level is the bottom limit for the PWM value, or in this case the minimum motor power. It is indexed by channel and the value is
* the range that comes from the main controller, before any processing into a timer value.
* the top level abstract class AbstractMotorControl contains these values.<p/>
* Created: 10/16/2020 12:40:41 PM
* @author Jonathan Groff Copyright (C) NeoCoreTechs 2022
 *
 */
public class SplitBridgeDriver extends HBridgeDriver {
	private int[][] motorDriveB= new int[][]{{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000},{255,0,10000}};
	public SplitBridgeDriver(int maxPower) {
		super(maxPower);
		// TODO Auto-generated constructor stub
	}

}
