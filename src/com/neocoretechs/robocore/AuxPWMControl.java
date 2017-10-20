package com.neocoretechs.robocore;

import java.io.IOException;

import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;
/**
* Create or activate a persistent PWM pin (M45 P1-P255) with an attached power level (S0-S255) by writing the directive to the 
* attached microcontroller using the ByteSerialDataPort.
*/
public class AuxPWMControl {
	public void activateAux(int[] pinaction) throws IOException {
		ByteSerialDataPort.getInstance().writeLine("M45 P"+String.valueOf(pinaction[0])+" S"+String.valueOf(pinaction[1]));
	}
}