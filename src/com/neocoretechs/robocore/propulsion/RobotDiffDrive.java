package com.neocoretechs.robocore.propulsion;

import java.io.Serializable;
import java.util.concurrent.ConcurrentHashMap;

import com.neocoretechs.robocore.config.Props;
import com.neocoretechs.robocore.config.TypedWrapper;
import com.neocoretechs.robocore.propulsion.RobotDiffDriveInterface;

/**
* Configuration for generic robot differential drive.
* @author Jonathan Groff (C) NeoCoreTechs 2020,2021
*/
public class RobotDiffDrive implements RobotDiffDriveInterface, Serializable {
	private static final long serialVersionUID = 1L;
	int leftSlot, rightSlot, leftChannel, rightChannel, controllerAxisX, controllerAxisY;
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
	public RobotDiffDrive(int slotLeft, int leftChannel, int slotRight, int rightChannel, int controllerAxisX, int controllerAxisY, 
			int motorKp, int motorKd, int motorKi, int motorKo, int motorPIDRate) {
		this.leftSlot = slotLeft;
		this.leftChannel = leftChannel;
		this.rightSlot = slotRight;
		this.rightChannel = rightChannel;
		leftWheel = new RobotWheel(wheelDiameter, ticksPerRevolution, motorKp, motorKd, motorKi, motorKo, motorPIDRate);
		rightWheel = new RobotWheel(wheelDiameter, ticksPerRevolution, motorKp, motorKd, motorKi, motorKo, motorPIDRate); 
	}
	public RobotDiffDrive(TypedWrapper[] lUN, TypedWrapper[] aXIS, TypedWrapper[] pID) {
		// TODO Auto-generated constructor stub
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
	public int getControllerLeftSlot() {
		return leftSlot;
	}

	@Override
	public int getControllerRightSlot() {
		return rightSlot;
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
		return String.format("Controller LeftSlot=%d,Left Channel=%d,RightSlot=%dRight Channel=%d,Wheel Track=%f,Indoor=%b\r\nLeft Wheel: %s\r\nRight Wheel: %s",
				leftSlot, leftChannel, rightSlot, rightChannel, wheelTrack, indoor,
				leftWheel == null ? "NULL" : leftWheel.toString(), rightWheel == null ? "NULL" : rightWheel.toString());
	}
	
	@Override
	public int getControllerAxisX() {
		 return controllerAxisX;
	}
	
	@Override
	public int getControllerAxisY() {
		 return controllerAxisY;
	}

}
