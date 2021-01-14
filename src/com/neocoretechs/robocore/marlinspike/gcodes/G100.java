package com.neocoretechs.robocore.marlinspike.gcodes;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;

public class G100 extends AbstractBasicResponse {
	private boolean DEBUG;
	public G100(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.G100.val(), 2); 
	}


}
