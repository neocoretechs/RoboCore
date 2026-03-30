package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * M15 [Z&lt;slot&gt;] C&lt;channel&gt; P&lt;pin&gt; S&lt;pin state 0 low, 1 high&gt; N&lt;number of counts before interrupt generated&gt;
 * Create digital encoder for controller at slot and channel.
 * Activate interrupt at S pin state.
 * Detect range N times before interrupt
 * @author Jonathan Groff (C) NeoCoreTechs 2022
 *
 */
public class M15 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M15(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M15.val());
	}

}
