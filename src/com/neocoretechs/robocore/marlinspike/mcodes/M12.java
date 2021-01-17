package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
/**
 * M12 [Z<slot>] C<channel> [P<offset>] [X<offset>]  set amount to add to G5 for min motor power, or X PWM level, If X, slot is PWM
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M12 extends AbstractBasicResponse {
	public M12(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M12.val());
	}
}
