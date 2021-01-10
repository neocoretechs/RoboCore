package com.neocoretechs.robocore.marlinspike.mcodes;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * M2 [Z<slot>] [C<channel> W<encoder pin> E<default dir>] <p/> 
 * Set smart controller (default) with optional encoder pin per channel, can be issued multiple times.
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M2 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M2(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.M2.val());
	}
}