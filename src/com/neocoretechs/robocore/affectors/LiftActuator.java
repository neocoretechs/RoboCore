package com.neocoretechs.robocore.affectors;

import java.io.Serializable;

import com.neocoretechs.robocore.config.Props;
import com.neocoretechs.robocore.config.TypedWrapper;

public class LiftActuator implements LiftActuatorInterface, Serializable {
	private static final long serialVersionUID = 1L;
	int axis;
	int channel;
	int slot;
	float componentUp;
	float componentDown;
	public LiftActuator(TypedWrapper[] lUN, TypedWrapper[] aXIS2, TypedWrapper[] pID) {
		//axis = Props.toInt(getControllerAxisPropertyName());
		//channel = Props.toInt(getControllerAxisPropertyName()+"Channel");
		//slot = Props.toInt(getControllerAxisPropertyName()+"Slot");
		//componentUp = Props.toFloat(getControllerAxisPropertyName()+"ComponentUp");
		//componentDown = Props.toFloat(getControllerAxisPropertyName()+"ComponentDown");
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
		return " Up="+componentUp+",Down="+componentDown+",slot="+slot+",channel="+channel;
	}

}
