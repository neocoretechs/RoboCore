package com.neocoretechs.robocore.affectors;

import com.neocoretechs.robocore.config.Props;

public class LEDIlluminator implements LEDIlluminatorInterface {

	public LEDIlluminator() {}

	@Override
	public String getControllerAxisPropertyName() {
		return "LEDCameraIlluminatorControl";
	}

	@Override
	public int getControllerAxis() {
		return Props.toInt(getControllerAxisPropertyName());
	}

	@Override
	public int getControllerChannel() {
		return Props.toInt(getControllerAxisPropertyName()+"Channel");
	}

	@Override
	public int getControllerSlot() {
		return Props.toInt(getControllerAxisPropertyName()+"Slot");
	}

}
