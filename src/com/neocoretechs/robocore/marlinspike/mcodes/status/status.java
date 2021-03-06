package com.neocoretechs.robocore.marlinspike.mcodes.status;


import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.marlinspike.AbstractBasicDataLoader;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * M700 - return controller status, this is the response
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class status extends AbstractBasicDataLoader {
	private boolean DEBUG = false;
	public status(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.STATUS.val(), 64);
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
