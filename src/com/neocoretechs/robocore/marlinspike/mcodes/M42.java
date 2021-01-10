package com.neocoretechs.robocore.marlinspike.mcodes;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * M42 - Create persistent digital pin, Write digital pin LOW P<pin> (This gives you a grounded pin)
 * @author Jonathan Groff (C) NeoCoreTEchs 2020,2021
 *
 */
public class M42 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M42(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.M42.val());
	}
}
