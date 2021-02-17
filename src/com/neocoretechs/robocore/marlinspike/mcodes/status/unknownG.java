package com.neocoretechs.robocore.marlinspike.mcodes.status;

import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.marlinspike.AbstractBasicDataLoader;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * unknown G code
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class unknownG extends AbstractBasicDataLoader {
	private boolean DEBUG = false;
	public unknownG(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.UNKNOWNG.val(), 2); 
		//
		// UNNOWNG error
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
