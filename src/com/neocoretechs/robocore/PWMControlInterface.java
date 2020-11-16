package com.neocoretechs.robocore;

import java.io.IOException;

/**
 * Interface to interface with PWM controller subsystem on the remote microcontroller.
 * @author groff
 *
 */
public interface PWMControlInterface {
	/**
	 * 
	 * @param slot1 slot
	 * @param channel2 channel
	 * @param valch3 value
	 * @throws IOException
	 */
	public void setAbsolutePWMLevel(int slot1, int channel2, int valch3) throws IOException;
}
