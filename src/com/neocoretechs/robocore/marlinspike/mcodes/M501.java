package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
/**
 * M501 Read settings from EEPROM
 *
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class M501 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M501(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M501.val());
	}
}
