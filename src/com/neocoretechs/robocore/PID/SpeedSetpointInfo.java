package com.neocoretechs.robocore.PID;

import com.neocoretechs.robocore.propulsion.DrivenWheelInterface;

/**
 * Speed in meters/second used as setpoint for PID processing.
 * @author Jonathan Groff (C) NeoCoreTechs 20202
 *
 */
public class SpeedSetpointInfo implements SetpointInfoInterface {
	float MAXIMUM = 1000.0f;
	float velocity, targetVelocity, prevErr;
	/**
	 * Convert meters per second to ticks per time frame.
	 * cpr = ticksPerRevolution
	 * ticksPerMeter = (float) (cpr / (Math.PI * wheelDiameter));
	 * ticksPerMeter = ticksPerRevolution / wheel Circumference
	 * SpeedToTicks = velocity * ticksPerMeter
	 * @param v
	 * @return
	 */
	public int SpeedToTicks( PIDParameterInterface ppi, DrivenWheelInterface dwi) {
		return (int) (velocity * dwi.getTickSetpointInfo().getTarget() / (ppi.getPIDRate() * Math.PI * dwi.getTickSetpointInfo().getWheelDiameter()));
	}
	@Override
	public void setTarget(float t) { velocity = t;}
	@Override
	public float getTarget() {	return velocity; }
	@Override
	public void setDesiredTarget(float t) {	targetVelocity = t;}
	@Override
	public float getDesiredTarget() {	return targetVelocity; }
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
}
