package com.neocoretechs.robocore.PID;
/**
 * Speed in meters/second used as setpoint for PID processing.
 * @author Jonathan Groff (C) NeoCoreTechs 20202
 *
 */
public class SpeedSetpointInfo implements SetpointInfoInterface {
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
	public int SpeedToTicks( PIDParameterInterface ppi, TickSetpointInfo tsi) {
		return (int) (velocity * tsi.getTarget() / (ppi.getPIDRate() * Math.PI * tsi.getWheelDiameter()));
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
}
