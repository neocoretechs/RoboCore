package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
/**
 * M301 P<pin> - attach ultrasonic device to pin
 * wont assign pin 0 as its sensitive
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class M301 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M301(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M301.val());
	}
}
