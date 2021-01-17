package com.neocoretechs.robocore.marlinspike.mcodes;


import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
/**
 * M39 P<pin> - Remove Persistent Analog pin 
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M39 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M39(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M39.val());
	}

}
