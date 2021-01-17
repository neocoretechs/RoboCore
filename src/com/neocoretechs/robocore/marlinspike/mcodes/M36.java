package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
/**
 * M36 - Clear all analog pins
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M36 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M36(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M36.val());
	}
}
