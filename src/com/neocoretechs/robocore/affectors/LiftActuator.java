package com.neocoretechs.robocore.affectors;

import com.neocoretechs.robocore.config.Props;

public class LiftActuator implements LiftActuatorInterface {
	int axis;
	int channel;
	int slot;
	float componentUp;
	float componentDown;
	public LiftActuator() {
		axis = Props.toInt(getControllerAxisPropertyName());
		channel = Props.toInt(getControllerAxisPropertyName()+"Channel");
		slot = Props.toInt(getControllerAxisPropertyName()+"Slot");
		componentUp = Props.toFloat(getControllerAxisPropertyName()+"ComponentUp");
		componentUp = Props.toFloat(getControllerAxisPropertyName()+"ComponentDown");
	}
	
	@Override
	public String getControllerAxisPropertyName() {
		return "LiftActuatorControl";
	}

	@Override
	public int getControllerAxis() {
		return axis;
	}

	@Override
	public int getControllerChannel() {
		return channel;
	}

	@Override
	public int getControllerSlot() {
		return slot;
	}

	@Override
	public float getControllerComponentUp() {
		return componentUp;
	}

	@Override
	public float getControllerComponentDown() {
		return componentDown;
	}
	
	@Override
	public String toString() {
		return getControllerAxisPropertyName()+" Up="+componentUp+",Down="+componentDown+",slot="+slot+",channel="+channel;
	}

}
