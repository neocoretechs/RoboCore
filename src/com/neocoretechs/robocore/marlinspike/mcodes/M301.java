package com.neocoretechs.robocore.marlinspike.mcodes;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * M301 P<pin> - attach ultrasonic device to pin
 * wont assign pin 0 as its sensitive
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class M301 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M301(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.M301.val());
	}
}
