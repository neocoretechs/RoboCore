package com.neocoretechs.robocore.marlinspike.mcodes;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
/**
 * M306 P<pin> T<target> [U] - toggle digital read, 0 or 1 for target value, default 0 optional INPUT_PULLUP 
 * Looks for target value, if so publish with <digitalpin> header and 1 - pin 2 - value
 *
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class M306 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M306(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.M306.val());
	}

}
