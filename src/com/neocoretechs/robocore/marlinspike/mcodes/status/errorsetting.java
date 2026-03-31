package com.neocoretechs.robocore.marlinspike.mcodes.status;


import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.marlinspike.AbstractBasicDataLoader;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * General error report.
 * @author Jonathan Groff (C) NeoCoreTechs 2026
 *
 */
public class errorsetting extends AbstractBasicDataLoader {
	private boolean DEBUG = false;

	public errorsetting(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.ERROR.val(), 16);
		//
		// Any type of code, message ends in ERROR
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
