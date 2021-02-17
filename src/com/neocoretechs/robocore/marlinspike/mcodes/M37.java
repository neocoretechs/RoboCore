package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * M37 - Clear all PWM pins, ALL MOTOR AND PWM DISABLED, perhaps not cleanly
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M37 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M37(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M37.val());
	}
}
