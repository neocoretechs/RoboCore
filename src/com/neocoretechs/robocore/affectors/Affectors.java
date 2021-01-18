package com.neocoretechs.robocore.affectors;
/**
 * Interface for peripheral affectors. Devices other than propulsion, typically motorized but
 * also including LEDs and other PWM driven devices.
 * @author Jonathan Groff (C) NeoCorTechs 2021
 *
 */
public class Affectors implements AffectorInterface {
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
