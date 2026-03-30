package com.neocoretechs.robocore.marlinspike.mcodes;


import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
* Split bridge or 2 half bridge motor controller. Takes 2 inputs: one for forward,called P, one for backward,called Q, then motor channel, 
* and then D, an enable pin. Finally, W&lt;encoder pin&gt;  to receive hall wheel sensor signals.
* Everything derived from HBridgeDriver can be done here.
* M4 [Z&lt;slot&gt;] P&lt;pin&gt; Q&lt;pin&gt; C&lt;channel&gt; D&lt;enable pin&gt; E&lt;default dir&gt; W&lt;encoder pin&gt;
* @author Jonathan Groff (C) NeoCoreTechs 2020,2021
*
*/
public class M4 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M4(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M4.val());
	}
}
