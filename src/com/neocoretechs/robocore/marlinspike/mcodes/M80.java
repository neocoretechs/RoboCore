package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * M80 - Turn on Power Supply
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class M80 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M80(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M80.val());
	}
}
