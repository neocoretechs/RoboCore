package com.neocoretechs.robocore.propulsion;

import com.neocoretechs.robocore.PID.TickSetpointInfo;

/**
* Configuration for ROSCOE2 robot
*
*/
public class ROSCOE2 implements RobotDiffDriveInterface {
	/* Define the robot parameters */
	public static boolean indoor = false; // div power by ten indoor mode
	//public static int MAXOUTPUT = 50; // indoor
	// so ticks are IMU data in mm/s, I hope. In static form its locked to diameter but here, as IMU data, 
	// it is variable based on desired speed.
	public static final int cpr = 500; // ticks per revolution, if IMU reports .5 m/s as 500 mm/s then 500, else we have to dynamically change based on speed
	public static float wheelDiameter = 203.2f; // millimeters, 8"
	public static float wheelTrack = 457.2f; // millimeters, 18"
	//public static float ticksPerMeter = (float) (cpr / (Math.PI * wheelDiameter));
	public static TickSetpointInfo tickSetpointInfo = new TickSetpointInfo(wheelDiameter, cpr);
	DrivenWheelInterface leftWheel;
	DrivenWheelInterface rightWheel;
	@Override
	public DrivenWheelInterface getLeftWheel() {
		return leftWheel;
	}

	@Override
	public DrivenWheelInterface getRightWheel() {
		return rightWheel;
	}

	@Override
	public int getLeftWheelChannel() {
		return 1;
	}
	@Override
	public int getRightWheelChannel() {
		return 2;
	}
	@Override
	public int getControllerSlot() {
		return 0;
	}
	@Override
	public void setDriveWheels(DrivenWheelInterface leftWheel, DrivenWheelInterface rightWheel) {
		this.leftWheel = leftWheel;
		this.rightWheel = rightWheel;	
	}
		
}
