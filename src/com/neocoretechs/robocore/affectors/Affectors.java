package com.neocoretechs.robocore.affectors;

public class Affectors implements AffectorInterface {
	BoomActuatorInterface boomActuator;
	LEDIlluminatorInterface LEDIlluminator;
	public Affectors() {
		boomActuator = new BoomActuator();
		LEDIlluminator = new LEDIlluminator();
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
		return boomActuator+" "+LEDIlluminator;
	}

}
