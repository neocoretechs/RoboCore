package com.neocoretechs.robocore.marlinspike.mcodes.status;


import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicDataLoader;
/**
 * General firmware report.
 * @author Jonathan Groff (C) NeoCoreTEchs 2020,2021
 *
 */
public class M115 extends AbstractBasicDataLoader {
	private boolean DEBUG = false;

	public M115(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer,  topicNames.M115.val(), 16);
		//
		// M115
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
