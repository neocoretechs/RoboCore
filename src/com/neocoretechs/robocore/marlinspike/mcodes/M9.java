package com.neocoretechs.robocore.marlinspike.mcodes;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
/**
 * Its the responsibility of each consumer to consume initial passed line from demuxer.
 * @author Jonathan Groff (C) NeoCoreTechs 2020,20201
 *
 */
public class M9 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M9(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.M9.val());
	}
}
