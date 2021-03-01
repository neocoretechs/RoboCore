package com.neocoretechs.robocore;

import java.io.IOException;

import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.MarlinspikeControlInterface;

/**
 * Create a persistent GPIO pin set high (M41) or low (M42) by writing the directive to the 
 * attached microcontroller using the AsynchDemuxer.
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class AuxGPIOControl {
	/**
	 * Queue an outbound write to the Marlinspike associated with the passed demuxer. 
	 * @param ad The AsynchDemuxer
	 * @param pinaction The pin action 2 element array; first element 0 is for pin number, element 1 for pin high 0, 1 for pin low
	 * @throws IOException If the demuxer wont at first mux.
	 */
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

	public void activateAux(MarlinspikeControlInterface marlinspikeControlInterface, int[] data) {
		// TODO Auto-generated method stub
		
	}
}
