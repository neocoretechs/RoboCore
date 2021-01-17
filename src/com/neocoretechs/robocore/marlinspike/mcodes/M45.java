package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
/**
 * PWM value between 0 and 255, default timer mode is 2; clear on match, default resolution is 8 bits, default prescale is 1
 * Prescale: 1,2,4,6,7,8,9 = none, 8, 64, 256, 1024, external falling, external rising
 * Use M445 to disable pin permanently or use timer more 0 to stop pulse without removing pin assignment
 * M45 - set up PWM P<pin> S<power val 0-255> [T<timer mode 0-3>] [R<resolution 8,9,10 bits>] [X<prescale 0-7>]
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M45 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M45(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M45.val());
	}
}
