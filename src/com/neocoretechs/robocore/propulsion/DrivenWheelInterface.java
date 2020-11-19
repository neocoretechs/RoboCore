package com.neocoretechs.robocore.propulsion;

import com.neocoretechs.robocore.PID.AbstractPIDController;
import com.neocoretechs.robocore.PID.SetpointInfoInterface;
import com.neocoretechs.robocore.PID.TickSetpointInfo;

/**
* class TwistInfo {
*	float yawDegrees;
*	float wheelTheta; change in theta calculated from wheels 
*	float imuTheta;  global theta from IMU, 2 PI radians 
*	float deltaTheta; change in theta calculated from IMU, current imu - prev imu 
*	float robotTheta bot heading in radians minus theta_offset 
*	float X;  bot X position in mm 
*	float Y; bot Y position in mm 
*	public String toString() {
*    return "TWIST Yaw d:"+yawDegrees+" Yaw target:"+wheelTheta+
*    		" IMU t:"+imuTheta+" Delta t:"+deltaTheta+" Robot t:"+robotTheta+" X:"+X+" Y:"+Y;
*	}
*}
*
*
*/
public interface DrivenWheelInterface {
	/* Define the robot parameters */
	public static boolean indoor = true; // div power by ten indoor mode

	
	public void setprevX(float t);
	public void setTwistInfo(TwistInfo t);
	public TwistInfo getTwistInfo();

	
	public AbstractPIDController getPIDController();
			
}
