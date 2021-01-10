package com.neocoretechs.robocore.marlinspike.mcodes;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * M40 P<pin> - Remove persistent digital pin 
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M40 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M40(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.M40.val());
	}
}
