package com.neocoretechs.robocore.marlinspike.mcodes;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * M302 P<pin> - remove ultrasonic pin
 *
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class M302 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M302(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.M302.val());
	}
}
