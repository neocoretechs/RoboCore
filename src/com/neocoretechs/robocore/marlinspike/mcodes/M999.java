package com.neocoretechs.robocore.marlinspike.mcodes;


import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
/**
 * M999 reset. 15ms delay after command, then suicide.
 *
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class M999 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M999(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M999.val());
	}
}
