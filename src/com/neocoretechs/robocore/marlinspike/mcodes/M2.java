package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
/**
 * M2 [Z<slot>] [C<channel> W<encoder pin> E<default dir>] <p/> 
 * Set smart controller (default) with optional encoder pin per channel, can be issued multiple times.
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M2 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M2(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M2.val());
	}
}