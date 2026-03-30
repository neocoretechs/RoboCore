package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * M80 - ENTER BOOTSEL MODE ON PICO - shuts down and allows new firmware to be copied to device
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021,2026
 *
 */
public class M80 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M80(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M80.val());
	}
}
