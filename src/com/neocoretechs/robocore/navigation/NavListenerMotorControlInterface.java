package com.neocoretechs.robocore.navigation;

import com.neocoretechs.robocore.propulsion.MotorControlInterface2D;

public interface NavListenerMotorControlInterface {

	public abstract void pushData(NavPacket np);

	public abstract MotorControlInterface2D getMotorControlListener();

	public abstract void setMotorControlListener(
			MotorControlInterface2D motorControlListener);

}