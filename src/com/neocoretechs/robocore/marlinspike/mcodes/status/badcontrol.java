package com.neocoretechs.robocore.marlinspike.mcodes.status;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicDataLoader;
/**
 * Bad motor control message, one line format
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class badcontrol extends AbstractBasicDataLoader {
	private boolean DEBUG = false;
	public badcontrol(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer,  topicNames.BADCONTROL.val(), 2);
		//
		// BADCONTROL error
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
