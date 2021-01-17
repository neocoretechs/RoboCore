package com.neocoretechs.robocore.marlinspike.mcodes.status;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicDataLoader;

/**
 * Unknown M code
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class unknownM extends AbstractBasicDataLoader {
	private boolean DEBUG = false;
	public unknownM(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.UNKNOWNM.val(), 2);
		//
		// UNNOWNM error
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
