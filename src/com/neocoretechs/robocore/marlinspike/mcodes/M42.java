package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
/**
 * M42 - Create persistent digital pin, Write digital pin LOW P<pin> (This gives you a grounded pin)
 * @author Jonathan Groff (C) NeoCoreTEchs 2020,2021
 *
 */
public class M42 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M42(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M42.val());
	}
}
