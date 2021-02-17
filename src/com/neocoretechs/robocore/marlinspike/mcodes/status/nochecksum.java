package com.neocoretechs.robocore.marlinspike.mcodes.status;


import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.marlinspike.AbstractBasicDataLoader;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * Missing line checksum
 * @author Jonathan Groff (C) NoeCoreTechs 2020,2021
 *
 */
public class nochecksum extends AbstractBasicDataLoader {
	private boolean DEBUG = false;
	public nochecksum(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.NOCHECKSUM.val(), 2);
		//
		// NOCHECKSUM error
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
