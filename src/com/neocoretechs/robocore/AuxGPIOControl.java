package com.neocoretechs.robocore;

import java.io.IOException;
import java.util.Arrays;

import com.neocoretechs.robocore.marlinspike.MarlinspikeControlInterface;

/**
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class AuxGPIOControl {

	/**
	 * Activate the GPIO server based on ServiceResponseBuilder from {@link MotionController}
	 * @param marlinspikeControl High level functions of the control
	 * @param data Request data
	 * @throws IOException 
	 */
	public void activateAux(MarlinspikeControlInterface marlinspikeControl, int[] data) throws IOException {
		System.out.printf("%s.activateAux() %s %s %s%n",this.getClass().getName(),marlinspikeControl.reportSystemId(),marlinspikeControl.reportAllControllerStatus(),Arrays.toString(data));
	}

}
