package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * Its the responsibility of each consumer to consume initial passed line from demuxer.
 * @author Jonathan Groff (C) NeoCoreTechs 2020,20201
 *
 */
public class M9 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M9(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M9.val());
	}
}
