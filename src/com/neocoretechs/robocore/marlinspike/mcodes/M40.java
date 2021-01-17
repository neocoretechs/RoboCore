package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
/**
 * M40 P<pin> - Remove persistent digital pin 
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M40 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M40(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M40.val());
	}
}
