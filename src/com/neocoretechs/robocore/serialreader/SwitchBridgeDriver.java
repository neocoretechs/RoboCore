package com.neocoretechs.robocore.serialreader;

import java.io.IOException;

import com.pi4j.io.gpio.GpioPinDigital;
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
	GpioPinDigital[] pdigitals;
	// 5 possible drive wheels, index is by channel-1.
	// motorDrive[channel] [[Digitals array index][dir pin]
	// PWM params array by channel:
	// 0-pin index to Digital pins array(default 255)
	// 1-direction pin
	int[][] motorDrive= new int[][]{{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0}};
	int[][] motorDriveB= new int[][]{{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0},{255,0}};
	int status_flag = 0;
	@Override
	public int commandMotorPower(int ch, int p) {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public int commandEmergencyStop(int status) throws IOException {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public int isConnected() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public String getDriverInfo(int ch) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public int queryFaultFlag() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public int queryStatusFlag() {
		// TODO Auto-generated method stub
		return 0;
	}

}
