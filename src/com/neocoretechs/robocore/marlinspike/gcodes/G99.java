package com.neocoretechs.robocore.marlinspike.gcodes;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * G99 start watchdog timer. G99 T<time_in_millis> values are 15,30,60,120,250,500,1000,4000,8000 default 4000
 * @author Jonathan Groff (c) NeoCoreTechs 2020,2021
 *
 */
public class G99 extends AbstractBasicResponse {
	private boolean DEBUG;
	AsynchDemuxer asynchDemuxer;
	String data;
	public G99(AsynchDemuxer asynchDemuxer) {
		//
		// G99
		//
		super(asynchDemuxer, topicNames.G99.val(), 2);
	}

}
