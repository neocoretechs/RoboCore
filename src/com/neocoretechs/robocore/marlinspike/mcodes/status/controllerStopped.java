package com.neocoretechs.robocore.marlinspike.mcodes.status;


import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.marlinspike.AbstractBasicDataLoader;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;

public class controllerStopped extends AbstractBasicDataLoader {
	private boolean DEBUG = false;
	public controllerStopped(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer,  topicNames.CONTROLLERSTOPPED.val(), 8);
		//
		// CONTROLLERSTOPPED error
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
