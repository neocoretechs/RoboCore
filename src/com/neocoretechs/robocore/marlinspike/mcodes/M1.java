package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * M1 - Marlinspike realtime output on
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M1 extends AbstractBasicResponse {
	public M1(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M1.val());
	}

}
