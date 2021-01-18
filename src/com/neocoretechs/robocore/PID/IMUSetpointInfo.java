package com.neocoretechs.robocore.PID;
/**
 * Target is yaw angle from IMU. 
 * Setpoint - Input. 
 * DesiredTarget is Setpoint, target is Input, delta returns difference.
 * @author Jonathan Groff (C) NeoCoreTechs 2020
 *
 */
public class IMUSetpointInfo implements SetpointInfoInterface {
	float MAXIMUM,MINIMUM;
	float yawAngle, desiredYawAngle, prevErr;
	@Override
	public void setTarget(float t) { yawAngle = t;	}
	@Override
	public float getTarget() { return yawAngle; }
	@Override
	public void setDesiredTarget(float t) { desiredYawAngle = t; }
	@Override
	public float getDesiredTarget() { return desiredYawAngle; }
	@Override
	public float delta() { return desiredYawAngle - yawAngle; }
	
	public static float yawDegrees(float yaw) {
		float yaw_degrees = (float) (yaw * 180.0 / Math.PI); // conversion to degrees
		if( yaw_degrees < 0 ) yaw_degrees += 360.0; // convert negative to positive angles
		return yaw_degrees;
	}
	@Override
	public void setPrevErr(float i) { prevErr = i;	}
	@Override
	public float getPrevErr() { return prevErr; }
	@Override
	public void setMaximum(float max) {MAXIMUM = max; }
	@Override
	public float getMaximum() { return MAXIMUM; }
	@Override
	public void setMinimum(float min) {MINIMUM = min; }
	@Override
	public float getMinimum() { return MINIMUM; }
	
	public String toString() {
		return "IMU Max Angle="+ MAXIMUM+",Min="+MINIMUM+",Yaw="+yawAngle+",Target Yaw="+desiredYawAngle+",Error="+prevErr;
	}
}
