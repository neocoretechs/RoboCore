package com.neocoretechs.robocore.marlinspike.mcodes.status;

import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.marlinspike.AbstractBasicDataLoader;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * Response to M701 which responds with digitalpinsetting header
 * @author groff
 *
 */
public class digitalpinsetting extends AbstractBasicDataLoader  {
	private boolean DEBUG = false;
	public digitalpinsetting(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer,  topicNames.DIGITALPINSETTING.val(), 16);
		//
		// M701 - DIGITALPINSETTING
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
