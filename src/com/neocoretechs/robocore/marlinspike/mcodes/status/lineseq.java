package com.neocoretechs.robocore.marlinspike.mcodes.status;


import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicDataLoader;
/**
 * Line sequence error
 * @author Jonathan Groff (C) NeoCoreTechs 202,2021
 *
 */
public class lineseq extends AbstractBasicDataLoader {
	private boolean DEBUG = false;

	public lineseq(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.LINESEQ.val(), 2);
		//
		// LINESEQ error
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
