package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * Switch bridge or 2 digital motor controller.<p>
 * Takes 2 inputs: one digital pin for forward,called P, one for backward,called Q, then motor channel,
 * and then D, an enable pin, and E default dir, with optional encoder.<br>
 * M5 Z&lt;slot&gt; P&lt;pin&gt; Q&lt;pin&gt; C&lt;channel&gt; D&lt;enable pin&gt; E&lt;default dir&gt; [W&lt;encoder&gt;]<br>
 * Create switch bridge Z slot, P forward pin, Q reverse pin, D enable, E default state of enable for dir.<p>
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M5 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M5(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M5.val());
	}

}
