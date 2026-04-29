package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.ActivationInterface;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.TypeSlotChannelEnable;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * M42 - Create persistent digital pin, Write digital pin LOW P&lt;pin&gt; (This gives you a grounded pin)
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M42 extends AbstractBasicResponse implements ActivationInterface {
	private boolean DEBUG;
	private TypeSlotChannelEnable tsce;
	
	public M42(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M42.val());
	}
	
	public M42(TypeSlotChannelEnable tsce) {
		this.tsce = tsce;
	}
	
	@Override
	public String getActivation(int... deviceLevel) {
		return String.format("M42 P%d S%dn", tsce.getPin(), deviceLevel[0]);
	}
}
