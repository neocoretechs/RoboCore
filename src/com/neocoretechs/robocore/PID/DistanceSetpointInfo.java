package com.neocoretechs.robocore.PID;

import java.io.Serializable;

/**
 * Set point information for PD control and other uses.
 * @author Jonathan Groff (C0 NeoCoreTechs 2020,2021
 *
 */
public class DistanceSetpointInfo implements SetpointInfoInterface, Serializable {
	private static final long serialVersionUID = 1L;
	private float MAXIMUM, MINIMUM;
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
	@Override
	public void setMinimum(float min) { MINIMUM = min;}
	@Override
	public float getMinimum() { return MINIMUM; }
	@Override
	public String toString() {
		return "Distance Max="+MAXIMUM+",Min="+MINIMUM+",Actual="+distance+",Target distance="+desiredDistance+",Error="+prevErr;
	}
}
