package com.neocoretechs.robocore.marlinspike.mcodes;


import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
/**
 * M799 [Z<controller>][X] Reset controller, if no argument, reset all. If X, slot is PWM
 *
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class M799 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M799(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer,topicNames.M799.val());
	}
}
