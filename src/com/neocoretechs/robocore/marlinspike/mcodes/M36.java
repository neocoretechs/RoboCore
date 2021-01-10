package com.neocoretechs.robocore.marlinspike.mcodes;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * M36 - Clear all analog pins
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M36 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M36(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.M36.val());
	}
}
