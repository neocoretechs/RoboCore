package com.neocoretechs.robocore.affectors;
/**
 * Interface for peripheral affectors. Devices other than propulsion, typically motorized but
 * also including LEDs and other PWM driven devices.
 * @author Jonathan Groff (C) NeoCorTechs 2021
 *
 */
public interface AffectorInterface {
	public BoomActuatorInterface getBoomActuatorInterface();
	public LEDIlluminatorInterface getLEDIlluminatorInterface();
	public LiftActuatorInterface getLiftActuatorInterface();
}
