package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * M14 [Z&lt;slot&gt;] C&lt;channel&gt; P&lt;pin&gt; L&lt;low range active&gt; H&lt;high range active&gt; N&lt;number of counts before interrupt generated&gt;
 * Create analog encoder for controller at slot and channel.
 * Activate interrupt between L low and H high range.
 * Detect range N times before interrupt
 * @author Jonathan Groff (C) NeoCoreTechs 2022
 */
public class M14 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M14(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M14.val());
	}

}
