package com.neocoretechs.robocore.marlinspike.mcodes.status;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.marlinspike.AbstractBasicDataLoader;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * Response to M706 which responds with assignedpins header
 * @author groff
 *
 */
public class assignedPins extends AbstractBasicDataLoader  {
	private boolean DEBUG = false;
	public assignedPins(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.ASSIGNEDPINS.val(), 16);
		//
		// M706 - ASSIGNEDPINS
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
