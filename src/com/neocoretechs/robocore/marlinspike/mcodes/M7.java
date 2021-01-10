package com.neocoretechs.robocore.marlinspike.mcodes;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * M7 [Z<slot>] [X]- Set motor override to stop motor operation, or optionally PWM operation, if X, slot is PWM
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M7 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M7(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.M7.val());
	}

}
