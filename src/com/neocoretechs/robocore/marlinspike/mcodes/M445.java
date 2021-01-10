package com.neocoretechs.robocore.marlinspike.mcodes;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * M445 P<pin> - Turn off pulsed write pin - disable PWM
 *
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class M445 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M445(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.M445.val());
	}
}
