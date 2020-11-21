package com.neocoretechs.robocore.propulsion;

/**
* Configuration for ROSCOE1 robot
*/
public class ROSCOE1 implements RobotDiffDriveInterface {
	int slot, leftChannel, rightChannel;
	/* Define the robot parameters */
	public static float wheelTrack = 304.8f; // millimeters, 12"
	public static boolean indoor = true; // div power by ten indoor mode
	private float wheelDiameter = 406.4f; // millimeters, 16"
	public int ticksPerRevolution = 500; // ticks per revolution, if IMU reports .5 m/s as 500 mm/s then 500, else we have to dynamically change based on speed
	DrivenWheelInterface leftWheel;
	DrivenWheelInterface rightWheel;
	//public static int MAXOUTPUT = 50; // indoor
	// so ticks are IMU data in mm/s, I hope. In static form its locked to diameter but here, as IMU data, 
	// it is variable based on desired speed.
	private TwistInfo twistinfo = new TwistInfo();
	public ROSCOE1(int slot, int leftChannel, int rightChannel, float wheelTrack) {
		this.slot = slot;
		this.leftChannel = leftChannel;
		this.rightChannel = rightChannel;
		leftWheel = new RobotWheel(wheelDiameter, ticksPerRevolution);
		rightWheel = new RobotWheel(wheelDiameter, ticksPerRevolution);
	}
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
		return leftChannel;
	}

	@Override
	public int getRightWheelChannel() {
		return rightChannel;
	}

	@Override
	public int getControllerSlot() {
		return slot;
	}

	@Override
	public void setDriveWheels(DrivenWheelInterface leftWheel, DrivenWheelInterface rightWheel) {
		this.leftWheel = leftWheel;
		this.rightWheel = rightWheel;	
	}

	@Override
	public float getWheelTrack() {
		return wheelTrack;
	}
	@Override
	public boolean isIndoor() {
		return indoor;
	}
}
