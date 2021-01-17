package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
/**
 * M6 [Z<slot>] [S<scale>] [X<scale>] 
 * Set motor or PWM scaling, divisor for final power to limit speed or level, set to 0 to cancel. If X, slot is PWM
 * @author Jonathan Groff (c) NeoCoreTechs 2020,2021
 *
 */
public class M6 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M6(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M6.val());
	}
}
