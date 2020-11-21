package com.neocoretechs.robocore.propulsion;

import com.neocoretechs.robocore.PID.AbstractPIDController;
import com.neocoretechs.robocore.PID.SpeedSetpointInfo;
import com.neocoretechs.robocore.PID.TickSetpointInfo;
/**
 * Represents one wheel of propulsion system with abstract
 * telemetry instances represented as setpoints that can be used for
 * PID control or general parameter storage.
 * @author Jonathan Groff (C) NeoCoreTechs 2020
 *
 */
public class RobotWheel implements DrivenWheelInterface {
	//public static int MAXOUTPUT = 50; // indoor
	// so ticks are IMU data in mm/s, I hope. In static form its locked to diameter but here, as IMU data, 
	// it is variable based on desired speed.
	private TickSetpointInfo tickSetpointInfo;
	private SpeedSetpointInfo speedSetpointInfo = new SpeedSetpointInfo();

	public RobotWheel(float wheelDiameter, int ticksPerRevolution) {
		tickSetpointInfo = new TickSetpointInfo(wheelDiameter, ticksPerRevolution);
	}

	@Override
	public void setprevX(float t) {
		// TODO Auto-generated method stub

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
		// TODO Auto-generated method stub
		return null;
	}
	
	// A struct to hold Odometry info 
	 class OdomInfo {
		long lastOdomTime;    // last millis() time odometry was calculated
		float linearX;	         // total linear x distance traveled
		float linearY;	         // total linear y distance traveled
		float angularZ;		 // total angular distance traveled
	}


}
