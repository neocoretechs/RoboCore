package com.neocoretechs.robocore.propulsion;
/**
 * Robot twist pose style positioning information.<p/>
 * Representation of positioning orientation in the reference frame.
 * @author Jonathan Groff (C) NeocoreTechs 2020,2021
 *
 */
public class TwistInfo {
	private float yawDegrees;
	private float wheelTheta;// change in theta calculated from wheels 
	private float imuTheta;  //global theta from IMU, 2 PI radians 
	private float deltaTheta;// change in theta calculated from IMU, current imu - prev imu 
	private float robotTheta;//bot heading in radians minus theta_offset 
	private float X;  		 //bot X position in mm 
	private float Y; 		//bot Y position in mm 
	public float getYawDegrees() {
		return yawDegrees;
	}
	public void setYawDegrees(float yawDegrees) {
		this.yawDegrees = yawDegrees;
	}
	public float getWheelTheta() {
		return wheelTheta;
	}
	public void setWheelTheta(float wheelTheta) {
		this.wheelTheta = wheelTheta;
	}
	public float getImuTheta() {
		return imuTheta;
	}
	public void setImuTheta(float imuTheta) {
		this.imuTheta = imuTheta;
	}
	public float getDeltaTheta() {
		return deltaTheta;
	}
	public void setDeltaTheta(float deltaTheta) {
		this.deltaTheta = deltaTheta;
	}
	public float getRobotTheta() {
		return robotTheta;
	}
	public void setRobotTheta(float robotTheta) {
		this.robotTheta = robotTheta;
	}
	public float getX() {
		return X;
	}
	public void setX(float x) {
		X = x;
	}
	public float getY() {
		return Y;
	}
	public void setY(float y) {
		Y = y;
	}
	public String toString() {
		return "TWIST Yaw degrees="+yawDegrees+",Yaw target="+wheelTheta+
	    		" IMU theta="+imuTheta+", Delta theta="+deltaTheta+",Robot theta"+robotTheta+",X="+X+",Y="+Y;
	}
	public TwistInfo() {}

}
