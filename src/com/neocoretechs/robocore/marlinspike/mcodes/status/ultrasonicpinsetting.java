package com.neocoretechs.robocore.marlinspike.mcodes.status;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * Response to M703 which responds with ultrasonicpinsetting header
 * @author groff
 *
 */
public class ultrasonicpinsetting extends AbstractBasicDataLoader  {
	private boolean DEBUG = false;
	public ultrasonicpinsetting(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.ULTRASONICPINSETTING.val(), 16);
		//
		// M703 - ULTRASONICPINSETTING
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
