package com.neocoretechs.robocore.propulsion;

import com.neocoretechs.robocore.PID.AbstractPIDController;
import com.neocoretechs.robocore.PID.SpeedSetpointInfo;
import com.neocoretechs.robocore.PID.TickSetpointInfo;

public class RobotWheel implements DrivenWheelInterface {
	//public static int MAXOUTPUT = 50; // indoor
	// so ticks are IMU data in mm/s, I hope. In static form its locked to diameter but here, as IMU data, 
	// it is variable based on desired speed.
	public static final int cpr = 500; // ticks per revolution, if IMU reports .5 m/s as 500 mm/s then 500, else we have to dynamically change based on speed
	public static float wheelDiameter = 406.4f; // millimeters, 16"
	public static float wheelTrack = 304.8f; // millimeters, 12"
	//public static float ticksPerMeter = (float) (cpr / (Math.PI * wheelDiameter));
	public static TickSetpointInfo tickSetpointInfo = new TickSetpointInfo(wheelDiameter, cpr);
	public static SpeedSetpointInfo speedSetpointInfo = new SpeedSetpointInfo();
	
	public RobotWheel() {
		// TODO Auto-generated constructor stub
	}

	@Override
	public void setprevX(float t) {
		// TODO Auto-generated method stub

	}

	@Override
	public void setTwistInfo(TwistInfo t) {
		// TODO Auto-generated method stub

	}

	@Override
	public TwistInfo getTwistInfo() {
		// TODO Auto-generated method stub
		return null;
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
