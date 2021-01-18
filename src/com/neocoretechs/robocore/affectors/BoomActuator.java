package com.neocoretechs.robocore.affectors;

import com.neocoretechs.robocore.config.Props;

public class BoomActuator implements BoomActuatorInterface {
	int x;
	int y;
	int slot;
	int channel;
	public BoomActuator() {
		x = Props.toInt(getControllerAxisPropertyName()+"X");
		y = Props.toInt(getControllerAxisPropertyName()+"Y");
		slot =  Props.toInt(getControllerAxisPropertyName()+"Slot");
		channel = Props.toInt(getControllerAxisPropertyName()+"Channel");
	}

	@Override
	public String getControllerAxisPropertyName() {
		return "BoomActuatorControl";
	}

	@Override
	public int getControllerAxisX() {
		return x;
	}

	@Override
	public int getControllerAxisY() {
		return y;
	}
	
	@Override
	public int getControllerSlot() {
		return slot;
	}
	
	@Override
	public int getControllerChannel() {
		return channel;
	}
	
	@Override
	public String toString() {
		return getControllerAxisPropertyName()+" x:"+x+" y:"+y+" slot:"+slot+" channel:"+channel;
	}

}
