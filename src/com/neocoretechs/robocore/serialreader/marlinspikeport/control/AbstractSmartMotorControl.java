package com.neocoretechs.robocore.serialreader.marlinspikeport.control;
/**
 * Differentiate smart motor control in the object model
 * @author groff
 *
 */
public abstract class AbstractSmartMotorControl extends AbstractMotorControl {

	public AbstractSmartMotorControl(int maxPower) {
		MAXMOTORPOWER = maxPower;
	}
}
