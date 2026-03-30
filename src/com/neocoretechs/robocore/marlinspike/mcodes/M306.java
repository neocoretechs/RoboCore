package com.neocoretechs.robocore.marlinspike.mcodes;


import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * M306 P&lt;pin&gt; T&lt;target&gt; [U] - toggle digital read, 0 or 1 for target value, default 0 optional INPUT_PULLUP 
 * Looks for target value, if so publish with &lt;digitalpin&gt; header and 1 - pin 2 - value
 *
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class M306 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M306(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer,topicNames.M306.val());
	}

}
