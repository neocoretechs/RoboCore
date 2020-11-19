package com.neocoretechs.robocore.propulsion;

/**
* Configuration for ROSCOE1 robot
*/
public class ROSCOE1 implements RobotDiffDriveInterface {
	/* Define the robot parameters */
	public static boolean indoor = true; // div power by ten indoor mode
	DrivenWheelInterface leftWheel;
	DrivenWheelInterface rightWheel;
	//public static int MAXOUTPUT = 50; // indoor
	// so ticks are IMU data in mm/s, I hope. In static form its locked to diameter but here, as IMU data, 
	// it is variable based on desired speed.		

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
