package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
/**
 * M302 P<pin> - remove ultrasonic pin
 *
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class M302 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M302(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M302.val());
	}
}
