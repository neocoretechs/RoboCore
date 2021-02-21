package com.neocoretechs.robocore.affectors;

import java.io.Serializable;

import com.neocoretechs.robocore.config.Props;
import com.neocoretechs.robocore.config.TypedWrapper;

public class LEDIlluminator implements LEDIlluminatorInterface, Serializable {
	private static final long serialVersionUID = 1L;
	int axis;
	int channel;
	int slot;
	public LEDIlluminator(TypedWrapper[] lUN, TypedWrapper[] aXIS2, TypedWrapper[] pID) {
		//axis = Props.toInt(getControllerAxisPropertyName());
		//channel = Props.toInt(getControllerAxisPropertyName()+"Channel");
		//slot = Props.toInt(getControllerAxisPropertyName()+"Slot");
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
	public String toString() {
		return " axis:"+axis+" slot:"+slot+" channel:"+channel;
	}

}
