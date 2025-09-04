package com.neocoretechs.robocore.marlinspike.gcodes;


import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.ActivationInterface;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.TypeSlotChannelEnable;
/**
 * G5 - Absolute command motor [Z<controller>] C<Channel> [P<motor power -1000 to 1000>] [X<PWM power -1000 to 1000>(scaled 0-2000)]
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class G5 extends AbstractBasicResponse implements ActivationInterface {
	private boolean DEBUG = false;
	private TypeSlotChannelEnable tsce; 
	public G5(AsynchDemuxer asynchDemuxer) {
		//
		// G5
		//
		super(asynchDemuxer, topicNames.G5.val(), 2);
	}
	public G5(TypeSlotChannelEnable tsce) {
		this.tsce = tsce;
	}
	@Override
	public String getActivation(int deviceLevel) {
		return String.format("G5 Z%d C%d P%d%n",tsce.getSlot(),tsce.getChannel(),deviceLevel);
	}

}
