package com.neocoretechs.robocore.marlinspike.mcodes;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
/**
 * Switch bridge or 2 digital motor controller.<p/>
 * Takes 2 inputs: one digital pin for forward,called P, one for backward,called Q, then motor channel,
 * and then D, an enable pin, and E default dir, with optional encoder.<br/>
 * M5 Z<slot> P<pin> Q<pin> C<channel> D<enable pin> E<default dir> [W<encoder>]<br/>
 * Create switch bridge Z slot, P forward pin, Q reverse pin, D enable, E default state of enable for dir.<p/>
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M5 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M5(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.M5.val());
	}

}
