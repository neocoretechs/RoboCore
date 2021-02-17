package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * M8 [Z<slot>][X] - Set motor override to start motor operation after stop override M7. If X, slot is PWM
 * @author Jonathan Groff (C) NeoCoreTechs 2020,20201
 *
 */
public class M8 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M8(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer,  topicNames.M8.val());
	}
}
