package com.neocoretechs.robocore.marlinspike.mcodes;


import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
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
