package com.neocoretechs.robocore.marlinspike.mcodes;


import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
/**
 * M502 Revert to default settings read from EEPROM
 *
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class M502 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M502(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M502.val());
	}

}
