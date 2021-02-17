package com.neocoretechs.robocore;

import java.io.IOException;

import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;
/**
 * Create a persistent GPIO pin set high (M41) or low (M42) by writing the directive to the 
 * attached microcontroller using the AsynchDemuxer.
 * @author jg
 *
 */
public class AuxGPIOControl {
	public void activateAux(AsynchDemuxer ad, int[] pinaction) throws IOException {
		switch(pinaction[1]) {
			case 0:
				AsynchDemuxer.addWrite(ad, "M41 P"+String.valueOf(pinaction[0]));
				break;
			case 1:
				AsynchDemuxer.addWrite(ad, "M42 P"+String.valueOf(pinaction[0]));
				break;
		}
	}
}
