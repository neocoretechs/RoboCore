package com.neocoretechs.robocore.marlinspike.mcodes;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * M5 [Z<slot>] [P<power>] [X<power>]- Set maximum motor power or optionally with X, a PWM control maximum level. If X, slot is PWM
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M5 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M5(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.M5.val());
	}

}
