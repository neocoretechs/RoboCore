package com.neocoretechs.robocore;

import java.io.IOException;

import com.microcaliperdevices.saje.io.machine.dataport.ByteSerialDataPort;

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
