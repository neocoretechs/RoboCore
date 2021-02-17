package com.neocoretechs.robocore.navigation;

import com.neocoretechs.robocore.marlinspike.MarlinspikeControlInterface;

public interface NavListenerMotorControlInterface {

	public abstract void pushData(NavPacket np);

	public abstract MarlinspikeControlInterface getMotorControlListener();

	public abstract void setMotorControlListener(
			MarlinspikeControlInterface motorControlListener);

}