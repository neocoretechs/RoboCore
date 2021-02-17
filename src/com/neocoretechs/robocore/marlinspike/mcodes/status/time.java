package com.neocoretechs.robocore.marlinspike.mcodes.status;


import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.marlinspike.AbstractBasicDataLoader;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * 
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class time extends AbstractBasicDataLoader {
	private boolean DEBUG = false;

	public time(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.TIME.val(), 4);
		//
		// TIME
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
