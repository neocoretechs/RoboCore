package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.ActivationInterface;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.TypeSlotChannelEnable;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 *
 * Use M445 to disable and remove pin
 * M45 - set up PWM P&lt;pin&gt; S&lt;power val 0-1000&gt; 
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M45 extends AbstractBasicResponse implements ActivationInterface {
	private boolean DEBUG;
	private TypeSlotChannelEnable tsce;
	
	public M45(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M45.val());
	}
	public M45(TypeSlotChannelEnable tsce) {
		this.tsce = tsce;
	}
	
	@Override
	public String getActivation(int... deviceLevel) {
		return "M45 P"+String.valueOf(deviceLevel[0])+" S"+String.valueOf(deviceLevel[1]);
	}
}
