package com.neocoretechs.robocore.affectors;

import java.io.Serializable;

import com.neocoretechs.robocore.config.Props;
import com.neocoretechs.robocore.config.TypedWrapper;
/**
 * Implementation for peripheral affector config. In this case the linear actuator 
 * driving a boom or robot arm or crane.
 * @author Jonathan Groff (C) NeoCorTechs 2021
 *
 */
public class BoomActuator implements BoomActuatorInterface, Serializable {
	private static final long serialVersionUID = 1L;
	int x;
	int y;
	int slot;
	int channel;
	public BoomActuator(TypedWrapper[] lUN, TypedWrapper[] aXIS, TypedWrapper[] pID) {
		//x = Props.toInt(getControllerAxisPropertyName()+"X");
		//y = Props.toInt(getControllerAxisPropertyName()+"Y");
		//slot =  Props.toInt(getControllerAxisPropertyName()+"Slot");
		//channel = Props.toInt(getControllerAxisPropertyName()+"Channel");
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
		return " x="+x+",y="+y+",slot="+slot+",channel="+channel;
	}

}
