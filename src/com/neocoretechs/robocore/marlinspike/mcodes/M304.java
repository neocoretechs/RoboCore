package com.neocoretechs.robocore.marlinspike.mcodes;


import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * M304 P&lt;pin&gt; [L&lt;min&gt;] [H&lt;max&gt;] [U] - toggle analog read optional INPUT_PULLUP with optional exclusion range 0-1024 via L&lt;min&gt; H&lt;max&gt;
 * if optional L and H values exclude readings in that range
 *
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class M304 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M304(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M304.val());
	}
}
