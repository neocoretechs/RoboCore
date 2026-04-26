package com.neocoretechs.robocore.marlinspike.gcodes;


import java.io.Serializable;

import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.ActivationInterface;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.TypeSlotChannelEnable;
/**
 * The list of channel power levels is a variable list depending on the number of channels the PWM device was configured with
 * G6 - Absolute command PWM [Z<controller>] [P<PWM power 0 to 2000>(scaled 0-2000)] [Q Power channel 2] [R Power channel 3...] [S] [T] [U] [V] [W] [X] [Y]
 * @author Jonathan Groff (C) NeoCoreTechs 2026
 *
 */
public class G6 extends AbstractBasicResponse implements ActivationInterface, Serializable {
	private static final long serialVersionUID = 1L;
	private boolean DEBUG = false;
	private TypeSlotChannelEnable tsce; 
	private static final String[] channels= {"P","Q","R","S","T","U","V","W","X","Y"};
	public G6(AsynchDemuxer asynchDemuxer) {
		//
		// G6
		//
		super(asynchDemuxer, topicNames.G6.val(), 2);
	}
	public G6(TypeSlotChannelEnable tsce) {
		this.tsce = tsce;
	}
	@Override
	public String getActivation(int... deviceLevel) {
		StringBuilder sb = new StringBuilder();
		for(int level = 0; level < deviceLevel.length; level++) {
			sb.append(channels[level]);
			sb.append(deviceLevel[level]);
			sb.append(" ");
		}
		return String.format("G6 Z%d %s%n",tsce.getSlot(),sb.toString());
	}

}
