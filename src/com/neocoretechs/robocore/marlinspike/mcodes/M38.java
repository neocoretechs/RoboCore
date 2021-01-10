package com.neocoretechs.robocore.marlinspike.mcodes;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * M38  P<pin> - Remove PWM pin, MOTOR AND PWM DISABLED, perhaps not cleanly
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M38 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M38(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.M38.val());
	}

}
