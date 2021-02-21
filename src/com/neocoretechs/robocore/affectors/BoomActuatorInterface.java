package com.neocoretechs.robocore.affectors;
/**
 * Interface for peripheral affector. In this case the linear actuator 
 * driving a boom or robot arm or crane.
 * @author Jonathan Groff (C) NeoCorTechs 2021
 *
 */
public interface BoomActuatorInterface {
	public int getControllerAxisX(); //derived by Props.toInt(getControllerAxisPropertyName()+"X");
	public int getControllerAxisY(); //derived by Props.toInt(getControllerAxisPropertyName()+"Y");
	public int getControllerChannel();
	public int getControllerSlot();
}
