package com.neocoretechs.robocore.marlinspike.mcodes;


import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
/**
 * M35 - Clear all digital pins
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M35 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M35(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M35.val());
	}
}
