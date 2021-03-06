package com.neocoretechs.robocore.propulsion;

import java.io.Serializable;

import com.neocoretechs.robocore.PID.AbstractPIDController;
import com.neocoretechs.robocore.PID.MotorPIDController;
import com.neocoretechs.robocore.PID.SpeedSetpointInfo;
import com.neocoretechs.robocore.PID.TickSetpointInfo;
/**
 * Represents one wheel of propulsion system with abstract
 * telemetry instances represented as setpoints that can be used for
 * PID control or general parameter storage.
 * @author Jonathan Groff (C) NeoCoreTechs 2020
 *
 */
public class RobotWheel implements DrivenWheelInterface, Serializable {
	private static final long serialVersionUID = 1L;
	//public static int MAXOUTPUT = 50; // indoor
	// so ticks are IMU data in mm/s, I hope. In static form its locked to diameter but here, as IMU data, 
	// it is variable based on desired speed.
	private String name;
	private TickSetpointInfo tickSetpointInfo;
	private SpeedSetpointInfo speedSetpointInfo;
	private MotorPIDController motorPIDController;
	private float x;
	
	public RobotWheel(String name, float wheelDiameter, float wheelTrack, int minimumSpeed, int maximumSpeed, int ticksPerRevolution, float kp, float ki, float kd, float ko, int pidRate) {
		this.name = name;
		speedSetpointInfo = new SpeedSetpointInfo(wheelTrack, minimumSpeed, maximumSpeed);
		tickSetpointInfo = new TickSetpointInfo(wheelDiameter, ticksPerRevolution);
		motorPIDController = new MotorPIDController(kp, ki, kd, ko, pidRate);
	}

	@Override
	public void setprevX(float t) {
		x = t;
	}

	@Override
	public void setTickSetpointInfo(TickSetpointInfo t) {
		tickSetpointInfo = t;		
	}

	@Override
	public TickSetpointInfo getTickSetpointInfo() {
		return tickSetpointInfo;
	}
	
	@Override
	public SpeedSetpointInfo getSpeedsetPointInfo() {
		return speedSetpointInfo;
	}
	
	@Override
	public AbstractPIDController getPIDController() {
		return motorPIDController;
	}
	
	public String toString() {
		return String.format("%s Tick Setpoint: %s\r\nSpeed Setpoint: %s\r\nPID Control: %s\r\n",
				name,
				tickSetpointInfo == null ? "NULL" : tickSetpointInfo.toString(),
				speedSetpointInfo == null ? "NULL" : speedSetpointInfo.toString(),
				motorPIDController == null ? "NULL" : motorPIDController.toString());
	}
	
	// A struct to hold Odometry info 
	 class OdomInfo {
		long lastOdomTime;    // last millis() time odometry was calculated
		float linearX;	         // total linear x distance traveled
		float linearY;	         // total linear y distance traveled
		float angularZ;		 // total angular distance traveled
	}


}
