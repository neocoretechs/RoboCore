package com.neocoretechs.robocore;

import java.io.IOException;

import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;

public class AuxPWMControl {
	public void activateAux(int[] pinaction) throws IOException {
		ByteSerialDataPort.getInstance().writeLine("M45 P"+String.valueOf(pinaction[0])+" S"+String.valueOf(pinaction[1]));
	}
}