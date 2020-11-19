package com.neocoretechs.robocore.PID;
/**
 * Target Setpoint for PID process
 * @author Jonathan Groff (C) NeoCoreTechs 2020
 *
 */
public interface SetpointInfoInterface {
	public void setPrevErr(float i);     // last error for integral
	public float getPrevErr();
	public void setTarget(float t);
	public float getTarget();
	public void setDesiredTarget(float t);
	public float getDesiredTarget();
	public float delta();
}
