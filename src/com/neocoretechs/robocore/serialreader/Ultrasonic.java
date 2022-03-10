package com.neocoretechs.robocore.serialreader;

import java.io.IOException;

/**
 * Allow for 1 or 2 pin ultrasonic placeholder, also for UART type with UltrasonicSerialDataPort. <p/>
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2022
 *
 */
public class Ultrasonic {
	boolean onePin = true;
	int txPin;
	int rxPin;
	int txrxPin;
	UltrasonicSerialDataPort port = null;
	public Ultrasonic(int pin) {
		txrxPin = pin;
	}
	public Ultrasonic(int tpin, int rpin) {
		txPin = tpin;
		rxPin = rpin;
		onePin = false;
	}
	public Ultrasonic() {
		this.port = UltrasonicSerialDataPort.getInstance();
	}
	public float getRange() throws IOException {
		if(port != null)
			port.readDistance();
		throw new IOException("Only UART based Ultrasonic sensor currently supported, and none found");
	}
}
