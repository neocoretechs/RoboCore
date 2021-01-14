package com.neocoretechs.robocore.marlinspike.gcodes;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * The responsibility of each of these consumers it to consume the first element
 * on the queue of lines read from the Marlinspike, be it the same as the passed
 * parameter 'peeked' into the retrieveData method, and continue to retrieve queue
 * elements until all elements relevant to this topic are thus consumed.<p/>
 * Many of these consumers follow the pattern of merely consuming the 'ack' from
 * a processed code.
 * G4 dwell
 * @author Jonathan N. Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class G4 extends AbstractBasicResponse {
	private boolean DEBUG = false;
	AsynchDemuxer asynchDemuxer;
	String data;
	public G4(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.G4.val());
		//
		// G4
		//
	}

}
