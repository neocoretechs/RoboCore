package com.neocoretechs.robocore.propulsion;

import java.io.Serializable;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Stream;

import com.neocoretechs.robocore.config.Props;
import com.neocoretechs.robocore.config.TypedWrapper;
import com.neocoretechs.robocore.propulsion.RobotDiffDriveInterface;

/**
* Configuration for generic robot differential drive.
* @author Jonathan Groff (C) NeoCoreTechs 2020,2021
*/
public class RobotDiffDrive implements RobotDiffDriveInterface, Serializable {
	public static boolean DEBUG = true;
	private static final long serialVersionUID = 1L;
	TypedWrapper[] LUN;
	TypedWrapper[] AXIS;
	TypedWrapper[] PID;
	int leftWheelLun = -1;
	int rightWheelLun = -1;
	int controllerAxisX = -1; 
	int controllerAxisY = -1;
	int leftSlot = -1;
	int leftChannel = -1;
	int rightSlot = -1;
	int rightChannel = -1;
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
	
	public RobotDiffDrive(TypedWrapper[] lUN, TypedWrapper[] aXIS, TypedWrapper[] pID) {
		this.LUN = lUN;
		this.AXIS = aXIS;
		this.PID = pID;
		// extract the wheel definitions from the logical units in config
		for(int i = 0; i < LUN.length; i++) {
			if(LUN[i].get("Name").equals("LeftWheel"))
					leftWheelLun = i;
			else
				if(LUN[i].get("Name").equals("RightWheel"))
					rightWheelLun = i;
			if(leftWheelLun != -1 && rightWheelLun != -1)
				break;
		}
		if(leftWheelLun == -1 || rightWheelLun == -1)
			throw new RuntimeException("Configuration is absent left or right wheel LUN declaration");
		setDrive();
	}
	
	private void setDrive() {
		Optional<Float> motorKpL = Optional.empty();
		Optional<Float> motorKdL = Optional.empty();
		Optional<Float> motorKiL = Optional.empty();
		Optional<Float> motorKoL = Optional.empty();
		Optional<Integer> motorPIDRateL = Optional.empty();
		Optional<Float> motorKpR = Optional.empty();
		Optional<Float> motorKdR = Optional.empty();
		Optional<Float> motorKiR = Optional.empty();
		Optional<Float> motorKoR = Optional.empty();
		Optional<Integer> motorPIDRateR = Optional.empty();

		motorKpL = Optional.ofNullable(PID[leftWheelLun].get("MotorKp")).filter(String.class::isInstance)
				.map(e -> Float.parseFloat((String) e));
		if(DEBUG)
			System.out.println("MotorKpL:"+motorKpL);
		motorKdL = Optional.ofNullable(PID[leftWheelLun].get("MotorKd")).filter(String.class::isInstance)
				.map(e -> Float.parseFloat((String) e));
		motorKiL = Optional.ofNullable(PID[leftWheelLun].get("MotorKi")).filter(String.class::isInstance)
				.map(e -> Float.parseFloat((String) e));
		motorKoL = Optional.ofNullable(PID[leftWheelLun].get("MotorKo")).filter(String.class::isInstance)
				.map(e -> Float.parseFloat((String) e));
		motorPIDRateL = Optional.ofNullable(PID[leftWheelLun].get("MotorPIDRate")).filter(String.class::isInstance)
		        .map(e -> Integer.parseInt((String) e));
		motorKpR= Optional.ofNullable(PID[rightWheelLun].get("MotorKp")).filter(String.class::isInstance)
				.map(e -> Float.parseFloat((String) e));
		motorKdR = Optional.ofNullable(PID[rightWheelLun].get("MotorKd")).filter(String.class::isInstance)
		        .map(e -> Float.parseFloat((String) e));
		motorKiR = Optional.ofNullable(PID[rightWheelLun].get("MotorKi")).filter(String.class::isInstance)
		        .map(e -> Float.parseFloat((String) e));
		motorKoR = Optional.ofNullable(PID[rightWheelLun].get("MotorKo")).filter(String.class::isInstance)
		        .map(e -> Float.parseFloat((String) e));
		motorPIDRateR = Optional.ofNullable(PID[rightWheelLun].get("MotorPIDRate")).filter(String.class::isInstance)
		        .map(e -> Integer.parseInt((String) e));
		try {
			leftWheel = new RobotWheel(wheelDiameter, ticksPerRevolution, motorKpL.get(), motorKdL.get(), motorKiL.get(), motorKoL.get(), motorPIDRateL.get());
		} catch(NoSuchElementException nse) {
			System.out.println("<<WARNING; USING PID DEFAULTS leftWheelLun: "+leftWheelLun+" PID[leftWheelLun]: "+
					(leftWheelLun < PID.length ? 
					PID[leftWheelLun]+(" "+" finally: "+(PID[leftWheelLun] != null ? 
					(PID[leftWheelLun].get("MotorKp")+","+PID[leftWheelLun].get("MotorKd")+","+PID[leftWheelLun].get("MotorKi")+","+PID[leftWheelLun].get("MotorKo")+","+PID[leftWheelLun].get("MotorPIDRate")):
						" COLLECTION NULL ")) :
						" PID ARRAY LEN BAD:"+PID.length));
			leftWheel = new RobotWheel(wheelDiameter, ticksPerRevolution, 1.0f, 1.0f, 1.0f, 1.0f, 1);
		}
		try {
			rightWheel = new RobotWheel(wheelDiameter, ticksPerRevolution, motorKpR.get(), motorKdR.get(), motorKiR.get(), motorKoR.get(), motorPIDRateR.get());
		} catch(NoSuchElementException nse) {
			System.out.println("<<WARNING USING PID DEFAULTS rightWheelLun: "+rightWheelLun+" PID[rightWheelLun]: "+
					(rightWheelLun < PID.length ? 
					PID[rightWheelLun]+(" "+" finally: "+(PID[rightWheelLun] != null ? 
					(PID[rightWheelLun].get("MotorKp")+","+PID[rightWheelLun].get("MotorKd")+","+PID[rightWheelLun].get("MotorKi")+","+PID[rightWheelLun].get("MotorKo")+","+PID[rightWheelLun].get("MotorPIDRate") ):
						" COLLECTION NULL ")) :
						" PID ARRAY LEN BAD:"+PID.length));
			rightWheel = new RobotWheel(wheelDiameter, ticksPerRevolution, 1.0f, 1.0f, 1.0f, 1.0f, 1);
		}
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
