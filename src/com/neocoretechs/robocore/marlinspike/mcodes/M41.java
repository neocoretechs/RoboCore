package com.neocoretechs.robocore.marlinspike.mcodes;


import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.ActivationInterface;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.TypeSlotChannelEnable;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * M41 - Create persistent digital pin, Write digital pin HIGH P&lt;pin&gt; (this gives you a 5v source on pin)
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M41 extends AbstractBasicResponse implements ActivationInterface {
	private boolean DEBUG;
	private TypeSlotChannelEnable tsce;
	public M41(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M41.val());
	}
	public M41(TypeSlotChannelEnable tsce) {
		this.tsce = tsce;
	}
	
	@Override
	public String getActivation(int... deviceLevel) {
		return String.format("M41 P%d S%dn", tsce.getPin(), deviceLevel[0]);
	}
}
