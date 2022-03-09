package com.neocoretechs.robocore.serialreader;

public abstract class AbstractPWMMotorControl extends AbstractMotorControl {

	public AbstractPWMMotorControl(int maxPower) {
		MAXMOTORPOWER = maxPower;
	}
	public abstract void resetMaxMotorPower();
}
