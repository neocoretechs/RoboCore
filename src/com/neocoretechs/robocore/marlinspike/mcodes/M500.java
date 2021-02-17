package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * M500 Store settings in EEPROM
 *
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class M500 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M500(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M500.val());
	}
}
