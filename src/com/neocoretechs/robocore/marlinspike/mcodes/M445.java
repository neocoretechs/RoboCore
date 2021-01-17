package com.neocoretechs.robocore.marlinspike.mcodes;


import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
/**
 * M445 P<pin> - Turn off pulsed write pin - disable PWM
 *
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class M445 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M445(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M445.val());
	}
}
