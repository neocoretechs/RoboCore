package com.neocoretechs.robocore.marlinspike.mcodes;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * M11 [Z<slot>] C<channel> [D<duration>] [X<duration>] - Set maximum cycle duration for given channel. If X, slot is PWM
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M11 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M11(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M11.val());
	}
}
