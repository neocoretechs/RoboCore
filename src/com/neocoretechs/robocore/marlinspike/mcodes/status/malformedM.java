package com.neocoretechs.robocore.marlinspike.mcodes.status;

import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.marlinspike.AbstractBasicDataLoader;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;

/**
 * Malformed M code
 * @author Jonathan Groff (C) NeoCoreTechs 2022
 *
 */
public class malformedM extends AbstractBasicDataLoader {
	private boolean DEBUG = false;
	public malformedM(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.MALFORMEDM.val(), 2);
		//
		// MalformedM error
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
