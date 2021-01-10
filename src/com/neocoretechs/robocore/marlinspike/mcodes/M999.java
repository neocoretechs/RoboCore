package com.neocoretechs.robocore.marlinspike.mcodes;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * M999 reset. 15ms delay after command, then suicide.
 *
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class M999 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M999(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.M999.val());
	}
}
