package com.neocoretechs.robocore.marlinspike.mcodes.status;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * M700 - return controller status, this is the response
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class status extends AbstractBasicDataLoader {
	private boolean DEBUG = true;
	public status(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.STATUS.val(), 64);
		//
		// M700
		//
	}
	@Override
	public Object getMachineReadingResult(MachineReading mr) {
		return mr.getReadingValString();
	}
	@Override
	public MachineReading formatMachineReading(String sdata) {
		return new MachineReading(sdata);
	}

}
