package com.neocoretechs.robocore.marlinspike.mcodes;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * M37 - Clear all PWM pins, ALL MOTOR AND PWM DISABLED, perhaps not cleanly
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M37 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M37(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.M37.val());
	}
}
