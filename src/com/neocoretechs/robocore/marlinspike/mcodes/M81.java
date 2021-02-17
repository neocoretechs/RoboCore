package com.neocoretechs.robocore.marlinspike.mcodes;


import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * M81 [Z<slot>] X - Turn off Power Z shut down motorcontroller in slot, X shut down PWM, slot -1 do all
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class M81 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M81(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer,topicNames.M81.val());
	}
}
