package com.neocoretechs.robocore.propulsion;

import java.io.Serializable;
import java.util.NoSuchElementException;
import java.util.Optional;
import com.neocoretechs.robocore.config.TypedWrapper;
import com.neocoretechs.robocore.propulsion.RobotDiffDriveInterface;

/**
* Configuration for generic robot differential drive.
* @author Jonathan Groff (C) NeoCoreTechs 2020,2021
*/
public class RobotDiffDrive implements RobotDiffDriveInterface, Serializable {
	public static boolean DEBUG = false;
	private static final long serialVersionUID = 1L;
	TypedWrapper[] LUN;
	TypedWrapper[] WHEEL;
	TypedWrapper[] AXIS;
	TypedWrapper[] PID;
	int leftWheelLun = -1;
	int rightWheelLun = -1;
	int controllerAxisX = -1; 
	int controllerAxisY = -1;
	/* Define the robot parameters */
	DrivenWheelInterface leftWheel;
	DrivenWheelInterface rightWheel;
	//public static int MAXOUTPUT = 50; // indoor
	// so ticks are IMU data in mm/s, I hope. In static form its locked to diameter but here, as IMU data, 
	// it is variable based on desired speed.
	private TwistInfo twistinfo = new TwistInfo();
	
	public RobotDiffDrive(TypedWrapper[] lUN, TypedWrapper[] wHEEL, TypedWrapper[] aXIS, TypedWrapper[] pID) {
		this.LUN = lUN;
		this.WHEEL = wHEEL;
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
		Optional<Float> leftWheelTrack = Optional.empty();// Props.toFloat("WheelTrackMM"); // millimeters
		Optional<Float> leftWheelDiameter = Optional.empty(); //Props.toFloat("WheelDiameterMM"); // millimeters, 16"
		Optional<Integer> leftTicksPerRevolution = Optional.empty();
		Optional<Float> rightWheelTrack = Optional.empty();// Props.toFloat("WheelTrackMM"); // millimeters
		Optional<Float> rightWheelDiameter = Optional.empty(); //Props.toFloat("WheelDiameterMM"); // millimeters, 16"
		Optional<Integer> rightTicksPerRevolution = Optional.empty(); 
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
		Optional<Integer> minSpeedL = Optional.empty();
		Optional<Integer> maxSpeedL = Optional.empty();
		Optional<Integer> minSpeedR = Optional.empty();
		Optional<Integer> maxSpeedR = Optional.empty();
		minSpeedL = Optional.ofNullable(LUN[leftWheelLun].get("MinValue")).filter(String.class::isInstance)
				.map(e -> Integer.parseInt((String) e));
		maxSpeedL = Optional.ofNullable(LUN[leftWheelLun].get("MaxValue")).filter(String.class::isInstance)
				.map(e -> Integer.parseInt((String) e));
		minSpeedR = Optional.ofNullable(LUN[rightWheelLun].get("MinValue")).filter(String.class::isInstance)
				.map(e -> Integer.parseInt((String) e));
		maxSpeedR = Optional.ofNullable(LUN[rightWheelLun].get("MaxValue")).filter(String.class::isInstance)
				.map(e -> Integer.parseInt((String) e));		
		leftWheelTrack = Optional.ofNullable(WHEEL[leftWheelLun].get("WheelTrackMM")).filter(String.class::isInstance)
				.map(e -> Float.parseFloat((String) e));
		leftWheelDiameter = Optional.ofNullable(WHEEL[leftWheelLun].get("WheelDiameterMM")).filter(String.class::isInstance)
				.map(e -> Float.parseFloat((String) e));
		leftTicksPerRevolution = Optional.ofNullable(WHEEL[leftWheelLun].get("TicksPerRevolution")).filter(String.class::isInstance)
				.map(e -> Integer.parseInt((String) e));
		rightWheelTrack = Optional.ofNullable(WHEEL[rightWheelLun].get("WheelTrackMM")).filter(String.class::isInstance)
				.map(e -> Float.parseFloat((String) e));
		rightWheelDiameter = Optional.ofNullable(WHEEL[rightWheelLun].get("WheelDiameterMM")).filter(String.class::isInstance)
				.map(e -> Float.parseFloat((String) e));
		rightTicksPerRevolution = Optional.ofNullable(WHEEL[rightWheelLun].get("TicksPerRevolution")).filter(String.class::isInstance)
				.map(e -> Integer.parseInt((String) e));
		motorKpL = Optional.ofNullable(PID[leftWheelLun].get("MotorKp")).filter(String.class::isInstance)
				.map(e -> Float.parseFloat((String) e));
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
			leftWheel = new RobotWheel((String) LUN[leftWheelLun].get("Name"), 
					leftWheelDiameter.get(), 
					leftWheelTrack.get(), 
					minSpeedL.get(), 
					maxSpeedL.get(), 
					leftTicksPerRevolution.get(), 
					motorKpL.get(), motorKdL.get(), 
					motorKiL.get(), motorKoL.get(),
					motorPIDRateL.get());
		} catch(NoSuchElementException nse) {
			System.out.println("<<WARNING; USING PID DEFAULTS leftWheelLun: "+leftWheelLun+" PID[leftWheelLun]: "+
					(leftWheelLun < PID.length ? 
					PID[leftWheelLun]+(" "+" finally: "+(PID[leftWheelLun] != null ? 
					(PID[leftWheelLun].get("MotorKp")+","+PID[leftWheelLun].get("MotorKd")+","+PID[leftWheelLun].get("MotorKi")+","+PID[leftWheelLun].get("MotorKo")+","+PID[leftWheelLun].get("MotorPIDRate")):
						" COLLECTION NULL ")) :
						" PID ARRAY LEN BAD:"+PID.length));
			leftWheel = new RobotWheel((String) LUN[leftWheelLun].get("Name"), 
					leftWheelDiameter.get(), 
					leftWheelTrack.get(), 
					minSpeedL.get(), 
					maxSpeedL.get(), 
					leftTicksPerRevolution.get(), 
					1.0f, 1.0f, 1.0f, 1.0f, 1);
		}
		try {
			rightWheel = new RobotWheel((String)LUN[rightWheelLun].get("Name"), 
					rightWheelDiameter.get(), 
					rightWheelTrack.get(), 
					minSpeedR.get(), 
					maxSpeedR.get(), 
					rightTicksPerRevolution.get(),
					motorKpR.get(),
					motorKdR.get(), 
					motorKiR.get(), 
					motorKoR.get(), 
					motorPIDRateR.get());
		} catch(NoSuchElementException nse) {
			System.out.println("<<WARNING USING PID DEFAULTS rightWheelLun: "+rightWheelLun+" PID[rightWheelLun]: "+
					(rightWheelLun < PID.length ? 
					PID[rightWheelLun]+(" "+" finally: "+(PID[rightWheelLun] != null ? 
					(PID[rightWheelLun].get("MotorKp")+","+PID[rightWheelLun].get("MotorKd")+","+PID[rightWheelLun].get("MotorKi")+","+PID[rightWheelLun].get("MotorKo")+","+PID[rightWheelLun].get("MotorPIDRate") ):
						" COLLECTION NULL ")) :
						" PID ARRAY LEN BAD:"+PID.length));
			rightWheel = new RobotWheel((String) LUN[leftWheelLun].get("Name"),
					rightWheelDiameter.get(), 
					rightWheelTrack.get(), 
					minSpeedR.get(), 
					maxSpeedR.get(), 
					rightTicksPerRevolution.get(), 1.0f, 1.0f, 1.0f, 1.0f, 1);
		}
		
		Optional<Integer> controllerAxisXO = Optional.ofNullable(AXIS[rightWheelLun].get("AxisX")).filter(String.class::isInstance)
		        .map(e -> Integer.parseInt((String) e));
		Optional<Integer> controllerAxisYO = Optional.ofNullable(AXIS[rightWheelLun].get("AxisY")).filter(String.class::isInstance)
		        .map(e -> Integer.parseInt((String) e));
		if(controllerAxisXO.isPresent()) {
			controllerAxisX = controllerAxisXO.get();
		} else {
			System.out.println("<<WARNING USING AXIS DEFAULTS rightWheelLun: "+rightWheelLun+" AXIS[rightWheelLun]: ");
			controllerAxisX = 0;
		}
		if(controllerAxisYO.isPresent()) {
			controllerAxisY = controllerAxisYO.get();
		} else {
			System.out.println("<<WARNING USING AXIS DEFAULTS rightWheelLun: "+rightWheelLun+" AXIS[rightWheelLun]: ");
			controllerAxisY = 2;
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
	public void setDriveWheels(DrivenWheelInterface leftWheel, DrivenWheelInterface rightWheel) {
		this.leftWheel = leftWheel;
		this.rightWheel = rightWheel;	
	}

	public String toString() {
		return String.format("%s\r\n%s",leftWheel == null ? "NULL" : leftWheel.toString(), rightWheel == null ? "NULL" : rightWheel.toString());
	}
	
	@Override
	public int getControllerAxisX() {
		 return controllerAxisX;
	}
	
	@Override
	public int getControllerAxisY() {
		 return controllerAxisY;
	}

	@Override
	public int getLeftwheelLun() {
		return leftWheelLun;
	}

	@Override
	public int getRightWheelLun() {
		return rightWheelLun;
	}

}
