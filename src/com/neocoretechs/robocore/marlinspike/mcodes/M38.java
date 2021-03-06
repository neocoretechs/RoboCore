package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * M38  P<pin> - Remove PWM pin, MOTOR AND PWM DISABLED, perhaps not cleanly
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M38 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M38(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer,  topicNames.M38.val());
	}

}
