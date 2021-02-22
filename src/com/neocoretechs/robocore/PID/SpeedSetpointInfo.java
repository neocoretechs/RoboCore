package com.neocoretechs.robocore.PID;

import java.io.Serializable;

import com.neocoretechs.robocore.propulsion.DrivenWheelInterface;

/**
 * Speed in meters/second used as setpoint for PID processing.
 * @author Jonathan Groff (C) NeoCoreTechs 20202
 *
 */
public class SpeedSetpointInfo implements SetpointInfoInterface, Serializable {
	private static final long serialVersionUID = 1L;
	float MAXIMUM = 1000.0f;
	float MINIMUM = 0.0f;
	float velocity, targetVelocity, prevErr;
	float wheelTrack;
	public SpeedSetpointInfo(float wheelTrack, int minimumSpeed, int maximumSpeed) {
		this.wheelTrack = wheelTrack;
		this.MINIMUM = minimumSpeed;
		this.MAXIMUM = maximumSpeed;
	}
	/**
	 * Convert meters per second to ticks per time frame.
	 * cpr = ticksPerRevolution
	 * ticksPerMeter = (float) (cpr / (Math.PI * wheelDiameter));
	 * ticksPerMeter = ticksPerRevolution / wheel Circumference
	 * SpeedToTicks = velocity * ticksPerMeter
	 * @param dwi the interface for the wheel
	 * @return 
	 */
	public int SpeedToTicks( DrivenWheelInterface dwi) {
		return (int) (velocity * dwi.getTickSetpointInfo().getTarget() / (dwi.getPIDController().getPIDRate() * Math.PI * dwi.getTickSetpointInfo().getWheelDiameter()));
	}
	@Override
	public void setTarget(float t) { velocity = t;}
	@Override
	public float getTarget() {	return velocity; }
	@Override
	public void setDesiredTarget(float t) {	targetVelocity = t;}
	@Override
	public float getDesiredTarget() {	return targetVelocity; }
	/**
	 * delta = Target - velocity
	 */
	@Override
	public float delta() {return targetVelocity - velocity; }
	@Override
	public void setPrevErr(float i) { prevErr = i; }
	@Override
	public float getPrevErr() { return prevErr; }
	@Override
	public void setMaximum(float max) { MAXIMUM = max; }
	@Override
	public float getMaximum() { return MAXIMUM; }
	@Override
	public void setMinimum(float min) { MINIMUM = min; }
	@Override
	public float getMinimum() { return MINIMUM; }

	public float getWheelTrack() { return wheelTrack; }
	
	public String toString() {
		return String.format("Speed Setpoint Info: Speed Max=%f, Min=%f, Velocity=%f, Target velocity=%f, WheelTrack=%f, Error=%f", MAXIMUM, MINIMUM, velocity, targetVelocity, wheelTrack,  prevErr);
	}
}
