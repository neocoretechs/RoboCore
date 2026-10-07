package com.neocoretechs.robocore.marlinspike.mcodes;

import java.io.Serializable;

import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * M6 
 * @author Jonathan Groff (c) NeoCoreTechs 2020,2021
 *
 */
public class M6 extends AbstractBasicResponse implements Serializable {
	private boolean DEBUG;
	public M6(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M6.val());
	}
}
