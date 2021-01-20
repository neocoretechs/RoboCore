package com.neocoretechs.robocore.affectors;

import java.io.Serializable;

/**
 * Interface for peripheral affectors. Devices other than propulsion, typically motorized but
 * also including LEDs and other PWM driven devices.
 * @author Jonathan Groff (C) NeoCorTechs 2021
 *
 */
public class Affectors implements AffectorInterface, Serializable {
	private static final long serialVersionUID = 1L;
	BoomActuatorInterface boomActuator;
	LEDIlluminatorInterface LEDIlluminator;
	LiftActuatorInterface liftActuator;
	public Affectors() {
		boomActuator = new BoomActuator();
		LEDIlluminator = new LEDIlluminator();
		liftActuator = new LiftActuator();
	}

	@Override
	public BoomActuatorInterface getBoomActuatorInterface() {
		return boomActuator;
	}

	@Override
	public LEDIlluminatorInterface getLEDIlluminatorInterface() {
		return LEDIlluminator;
	}
	
	@Override
	public String toString() {
		return boomActuator+"\r\n"+LEDIlluminator+"\r\n"+liftActuator;
	}

	@Override
	public LiftActuatorInterface getLiftActuatorInterface() {
		return liftActuator;
	}

}
