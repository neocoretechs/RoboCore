package com.neocoretechs.robocore.marlinspike.mcodes;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * M41 - Create persistent digital pin, Write digital pin HIGH P<pin> (this gives you a 5v source on pin)
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M41 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M41(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.M41.val());
	}
}
