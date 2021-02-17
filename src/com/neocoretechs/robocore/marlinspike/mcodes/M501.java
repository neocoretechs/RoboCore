package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
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
