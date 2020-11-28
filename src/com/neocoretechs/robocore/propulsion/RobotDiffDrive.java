package com.neocoretechs.robocore.propulsion;

import com.neocoretechs.robocore.config.Props;
import com.neocoretechs.robocore.propulsion.RobotDiffDriveInterface;

/**
* Configuration for generic robot differential drive
*/
public class RobotDiffDrive implements RobotDiffDriveInterface {
	int slot, leftChannel, rightChannel;
	/* Define the robot parameters */
	public static float wheelTrack = Props.toFloat("WheelTrackMM"); // millimeters
	public static boolean indoor = Props.toBoolean("IsIndoor"); // div power by ten indoor mode
	private float wheelDiameter = Props.toFloat("WheelDiameterMM"); // millimeters, 16"
	public int ticksPerRevolution = Props.toInt("TicksPerRevolution"); // ticks per revolution, if IMU reports .5 m/s as 500 mm/s then 500, else we have to dynamically change based on speed
	DrivenWheelInterface leftWheel;
	DrivenWheelInterface rightWheel;
	//public static int MAXOUTPUT = 50; // indoor
	// so ticks are IMU data in mm/s, I hope. In static form its locked to diameter but here, as IMU data, 
	// it is variable based on desired speed.
	private TwistInfo twistinfo = new TwistInfo();
	public RobotDiffDrive() {
		this.slot = Props.toInt("DriveControllerSlot");
		this.leftChannel = Props.toInt("LeftWheelChannel");
		this.rightChannel = Props.toInt("RightWheelChannel");
		leftWheel = new RobotWheel(wheelDiameter, ticksPerRevolution, Props.toFloat("MotorKp"), 
				Props.toFloat("MotorKd"), 
				Props.toFloat("MotorKi"), 
				Props.toFloat("MotorKo"), 
				Props.toInt("MotorPIDRate"));
		rightWheel = new RobotWheel(wheelDiameter, ticksPerRevolution, Props.toFloat("MotorKp"), 
				Props.toFloat("MotorKd"), 
				Props.toFloat("MotorKi"), 
				Props.toFloat("MotorKo"), 
				Props.toInt("MotorPIDRate"));
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
	
	public String toString() {
		return String.format("Controller Slot:%d Left Channel=%d Right Channel=%d Wheel Track=%f Indoor=%b\r\n"
				+ "Left Wheel=%s\r\nRight Wheel=%s\r\n",slot, leftChannel, rightChannel, wheelTrack, indoor,
				leftWheel == null ? "NULL" : leftWheel.toString(), rightWheel == null ? "NULL" : rightWheel.toString());
	}
}
