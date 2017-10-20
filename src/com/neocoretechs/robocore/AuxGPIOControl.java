package com.neocoretechs.robocore;

import java.io.IOException;

import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;
/**
 * Create a persistent GPIO pin set high (M41) or low (M42) by writing the directive to the 
 * attached microcontroller using the ByteSerialDataPort.
 * @author jg
 *
 */
public class AuxGPIOControl {
	public void activateAux(int[] pinaction) throws IOException {
		switch(pinaction[1]) {
			case 0:
				ByteSerialDataPort.getInstance().writeLine("M41 P"+String.valueOf(pinaction[0]));
				break;
			case 1:
				ByteSerialDataPort.getInstance().writeLine("M42 P"+String.valueOf(pinaction[0]));
				break;
		}
	}
}
