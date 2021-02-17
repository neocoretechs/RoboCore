package com.neocoretechs.robocore.marlinspike.mcodes;


import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * The responsibility of each of these consumers it to consume the first element
 * on the queue of lines read from the Marlinspike, be it the same as the passed
 * parameter 'peeked' into the retrieveData method, and continue to retrieve queue
 * elements until all elements relevant to this topic are thus consumed.<p/>
 * Many of these consumers follow the pattern of merely consuming the 'ack' from
 * a processed code.
 * M0 - Marlinspike realtime output off
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M0 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M0(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M0.val());
	}
}
