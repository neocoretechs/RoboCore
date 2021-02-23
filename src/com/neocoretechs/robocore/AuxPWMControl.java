package com.neocoretechs.robocore;

import java.io.IOException;

import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;

/**
* Create or activate a persistent PWM pin (M45 P1-P255) with an attached power level (S0-S255) by writing the directive to the 
* attached Marlinspike microcontroller using the ByteSerialDataPort encapsulated in the Demuxer.
* @author Jonathan Groff (C) NeoCoreTechs 2020,2021
*/
public class AuxPWMControl {
	/**
	 * Queue an outbound write to the Marlinspike using the demuxer and pinaction array.
	 * @param ad The AsynchDemuxer
	 * @param pinaction The array of actions for the pin. Array element 0 is the pin number, element 1 is power level 0-255,
	 * @throws IOException If there is a problem with the queued write request.
	 */
	public void activateAux(AsynchDemuxer ad, int[] pinaction) throws IOException {
		AsynchDemuxer.addWrite(ad,"M45 P"+String.valueOf(pinaction[0])+" S"+String.valueOf(pinaction[1]));
	}
}