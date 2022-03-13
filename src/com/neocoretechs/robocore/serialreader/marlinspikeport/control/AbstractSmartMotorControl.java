package com.neocoretechs.robocore.serialreader.marlinspikeport.control;

public abstract class AbstractSmartMotorControl extends AbstractMotorControl {

	public AbstractSmartMotorControl(int maxPower) {
		MAXMOTORPOWER = maxPower;
	}
}
