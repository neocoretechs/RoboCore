package com.neocoretechs.robocore.marlinspike.gcodes;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * G5 - Absolute command motor [Z<controller>] C<Channel> [P<motor power -1000 to 1000>] [X<PWM power -1000 to 1000>(scaled 0-2000)]
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class G5 extends AbstractBasicResponse {
	private boolean DEBUG;
	public G5(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		//
		// G5
		//
		super(asynchDemuxer, topics, topicNames.G5.val(), 2);
	}

}
