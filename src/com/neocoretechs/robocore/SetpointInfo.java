package com.neocoretechs.robocore;

public interface SetpointInfo {
	public static final int MAXOUTPUT = 1000; // normal
	public void setTargetSpeed(float t);            // target speed in m/s
	public void setTargetMM(float t);
	public void setTargetTicksPerFrame(float t);    // target speed in ticks per frame
	public float getTargetSpeed();            // target speed in m/s
	public float getTargetMM();
	public float getTargetTicksPerFrame();    // target speed in ticks per frame
	public int SpeedToTicks(float v);
}
