package com.neocoretechs.robocore.affectors;

import com.neocoretechs.robocore.config.Props;

public class BoomActuator implements BoomActuatorInterface {

	public BoomActuator() {}

	@Override
	public String getControllerAxisPropertyName() {
		return "BoomActuatorControl";
	}

	@Override
	public int getControllerAxisX() {
		return Props.toInt(getControllerAxisPropertyName()+"X");
	}

	@Override
	public int getControllerAxisY() {
		return Props.toInt(getControllerAxisPropertyName()+"Y");
	}
	
	@Override
	public int getControllerSlot() {
		return Props.toInt(getControllerAxisPropertyName()+"Slot");
	}
	
	@Override
	public int getControllerChannel() {
		return Props.toInt(getControllerAxisPropertyName()+"Channel");
	}

}
