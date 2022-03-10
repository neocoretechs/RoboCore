package com.neocoretechs.robocore.serialreader;

public abstract class AbstractSmartMotorControl extends AbstractMotorControl {

	public AbstractSmartMotorControl(int maxPower) {
		MAXMOTORPOWER = maxPower;
	}
}
