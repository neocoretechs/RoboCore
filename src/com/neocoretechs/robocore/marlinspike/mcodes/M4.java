package com.neocoretechs.robocore.marlinspike.mcodes;


import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
* Split bridge or 2 half bridge motor controller. Takes 2 inputs: one for forward,called P, one for backward,called Q, then motor channel, 
* and then D, an enable pin. Finally, W<encoder pin>  to receive hall wheel sensor signals and 
* optionally PWM timer setup [R<resolution 8,9,10 bits>] [X<prescale 0-7>].
* Everything derived from HBridgeDriver can be done here.
* M4 [Z<slot>] P<pin> Q<pin> C<channel> D<enable pin> E<default dir> W<encoder pin> [R<resolution 8,9,10 bits>] [X<prescale 0-7>]
* @author Jonathan Groff (C) NeoCoreTechs 2020,2021
*
*/
public class M4 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M4(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M4.val());
	}
}
