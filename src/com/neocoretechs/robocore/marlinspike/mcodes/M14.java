package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * M14 [Z<slot>] C<channel> P<pin> L<low range active> H<high range active> N<number of counts before interrupt generated>
 * Create analog encoder for controller at slot and channel.
 * Activate interrupt between L low and H high range.
 * Detect range N times before interrupt
 * @author Jonathan Groff (C) NeoCoreTechs 2022
 */
public class M14 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M14(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M14.val());
	}

}
