package com.neocoretechs.robocore.affectors;

public interface LEDIlluminatorInterface {
	public int getControllerAxis(); //derived by Props.toInt(getControllerAxisPropertyName());
	public int getControllerChannel();
	public int getControllerSlot();
}
