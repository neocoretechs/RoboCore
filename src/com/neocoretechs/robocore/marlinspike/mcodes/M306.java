package com.neocoretechs.robocore.marlinspike.mcodes;


import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * M306 P<pin> T<target> [U] - toggle digital read, 0 or 1 for target value, default 0 optional INPUT_PULLUP 
 * Looks for target value, if so publish with <digitalpin> header and 1 - pin 2 - value
 *
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class M306 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M306(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer,topicNames.M306.val());
	}

}
