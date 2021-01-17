package com.neocoretechs.robocore.marlinspike.mcodes;


import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
/**
 * M304 P<pin> [L<min>] [H<max>] [U] - toggle analog read optional INPUT_PULLUP with optional exclusion range 0-1024 via L<min> H<max>
 * if optional L and H values exclude readings in that range
 *
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class M304 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M304(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M304.val());
	}
}
