package com.neocoretechs.robocore.PID;
/**
 * Target is TicksPerFrame, or TicksPerMeter. Depends on wheel diameter.
 * cpr = ticksPerRevolution
 * ticksPerMeter = (float) (cpr / (Math.PI * wheelDiameter));
 * ticksPerMeter = ticksPerRevolution / wheel Circumference
 * SpeedToTicks = velocity * ticksPerMeter
 * All measurements are in MM, so to get per meter we have to scale it.
 * @author Jonathan Groff (C) NeoCoreTechs 2020
 *
 */
public class TickSetpointInfo extends DistanceSetpointInfo {
	float wheelDiameter, ticksPerRevolution;
	public float getWheelDiameter() { return wheelDiameter; }
	public void setWheelDiameter(float w) { wheelDiameter = w; }
	public float getTicksPerMeter() { return (float) (ticksPerRevolution / (Math.PI * wheelDiameter)) / 1000.0f; }
	/**
	 * In MM
	 * @param wheelDiameter
	 * @param ticksPerRevolution
	 */
	public TickSetpointInfo(float wheelDiameter, float ticksPerRevolution) {
		this.wheelDiameter = wheelDiameter;
		this.ticksPerRevolution = ticksPerRevolution;
	}
	
	public String toString() {
		return "Distance Max ticks="+getMaximum()+",Min="+getMinimum()+",Wheel diameter="+wheelDiameter+",Ticks per rev="+ticksPerRevolution+",Error="+getPrevErr();
	}
}
