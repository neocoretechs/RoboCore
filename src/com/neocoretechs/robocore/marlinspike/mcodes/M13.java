package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
/**
 * M13 [Z<slot>] [P<power>] [X<power>]- Set maximum motor power or optionally with X, a PWM control maximum level. If X, slot is PWM
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M13 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M13(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M13.val());
	}

}
