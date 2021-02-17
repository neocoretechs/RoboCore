package com.neocoretechs.robocore.marlinspike.mcodes.status;

import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.marlinspike.AbstractBasicDataLoader;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * Response to M702 which responds with analogpinsetting header
 * @author groff
 *
 */
public class analogpinsetting extends AbstractBasicDataLoader  {
	private boolean DEBUG = false;
	public analogpinsetting(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer,  topicNames.ANALOGPINSETTING.val(), 16);
		//
		// M702 - ANALOGPINSETTING
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
