package com.neocoretechs.robocore.marlinspike.mcodes;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * M12 [Z<slot>] C<channel> [P<offset>] [X<offset>]  set amount to add to G5 for min motor power, or X PWM level, If X, slot is PWM
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M12 extends AbstractBasicResponse {
	public M12(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.M12.val());
	}
}
