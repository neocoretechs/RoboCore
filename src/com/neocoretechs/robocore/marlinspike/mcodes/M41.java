package com.neocoretechs.robocore.marlinspike.mcodes;


import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * M41 - Create persistent digital pin, Write digital pin HIGH P<pin> (this gives you a 5v source on pin)
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M41 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M41(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M41.val());
	}
}
