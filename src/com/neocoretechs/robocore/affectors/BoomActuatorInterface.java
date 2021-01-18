package com.neocoretechs.robocore.affectors;

public interface BoomActuatorInterface {
	public String getControllerAxisPropertyName();
	public int getControllerAxisX(); //derived by Props.toInt(getControllerAxisPropertyName()+"X");
	public int getControllerAxisY(); //derived by Props.toInt(getControllerAxisPropertyName()+"Y");
	public int getControllerChannel();
	public int getControllerSlot();
}
