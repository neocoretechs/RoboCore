package com.neocoretechs.robocore.affectors;

import java.io.Serializable;

import com.neocoretechs.robocore.config.TypedWrapper;

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
	public Affectors(TypedWrapper[] lUN, TypedWrapper[] aXIS, TypedWrapper[] pID) {
		boomActuator = new BoomActuator(lUN, aXIS, pID);
		LEDIlluminator = new LEDIlluminator(lUN, aXIS, pID);
		liftActuator = new LiftActuator(lUN, aXIS, pID);
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
