package com.neocoretechs.robocore.affectors;
/**
 * This operates from a POV pad and so has a component value method. If we remove manual pad control
 * we can just address this directly as an additional parameter.
 * @author Jonathan Groff (C) NeoCoreTechs 2021
 *
 */
public interface LiftActuatorInterface {
	public int getControllerAxis(); //derived by Props.toInt(getControllerAxisPropertyName());
	public int getControllerChannel();
	public int getControllerSlot();
	public float getControllerComponentUp();
	public float getControllerComponentDown();
}
