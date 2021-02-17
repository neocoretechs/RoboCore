package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
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
