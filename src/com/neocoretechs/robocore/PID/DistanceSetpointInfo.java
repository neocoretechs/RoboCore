package com.neocoretechs.robocore.PID;

public class DistanceSetpointInfo implements SetpointInfoInterface {
	private float MAXIMUM;
	private float distance, desiredDistance, prevErr;
	@Override
	public void setTarget(float t) { distance = t; }
	@Override
	public float getTarget() {return distance; }
	@Override
	public void setDesiredTarget(float t) { desiredDistance = t;}
	@Override
	public float getDesiredTarget() {return desiredDistance;}
	@Override
	public float delta() { return desiredDistance - distance; }
	@Override
	public void setPrevErr(float i) { prevErr = i;}
	@Override
	public float getPrevErr() { return prevErr; }
	@Override
	public void setMaximum(float max) { MAXIMUM = max;}
	@Override
	public float getMaximum() { return MAXIMUM; }
	
	public String toString() {
		return "Distance Max="+MAXIMUM+" distance="+distance+" target distance="+desiredDistance+" err="+prevErr;
	}
}
