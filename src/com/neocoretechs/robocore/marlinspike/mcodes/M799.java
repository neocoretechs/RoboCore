package com.neocoretechs.robocore.marlinspike.mcodes;


import java.io.Serializable;

import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * M799 
 *
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class M799 extends AbstractBasicResponse implements Serializable {
	private boolean DEBUG;
	public M799(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer,topicNames.M799.val());
	}
}
