package com.neocoretechs.robocore.marlinspike.mcodes.status;


import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicDataLoader;
/**
 * No M or G code found for input line
 * @author Jonathan Groff 9C) NeoCoreTechs 2020,2021
 *
 */
public class noMorG extends AbstractBasicDataLoader{
	private boolean DEBUG = false;

	public noMorG(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer,  topicNames.NOMORGCODE.val(), 2);
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
