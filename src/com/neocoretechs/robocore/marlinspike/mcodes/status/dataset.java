package com.neocoretechs.robocore.marlinspike.mcodes.status;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicDataLoader;
/**
 * dataset - generic dataset return
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class dataset extends AbstractBasicDataLoader {
	private boolean DEBUG = false;
	public dataset(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.DATASET.val(), 64);
		//
		// DATASET
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
