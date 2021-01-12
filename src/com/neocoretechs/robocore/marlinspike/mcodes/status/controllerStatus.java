package com.neocoretechs.robocore.marlinspike.mcodes.status;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * Get the status of controllers<p/>
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class controllerStatus extends AbstractBasicDataLoader {
	private boolean DEBUG = false;
	public controllerStatus(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.CONTROLLERSTATUS.val(), 16);
		//
		// CONTROLLERSTATUS
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
