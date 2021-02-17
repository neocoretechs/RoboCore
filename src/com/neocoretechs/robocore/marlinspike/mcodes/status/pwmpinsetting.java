package com.neocoretechs.robocore.marlinspike.mcodes.status;

import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.marlinspike.AbstractBasicDataLoader;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * Response to M704 which responds with pwmpinsetting header
 * @author groff
 *
 */
public class pwmpinsetting extends AbstractBasicDataLoader  {
	private boolean DEBUG = false;
	public pwmpinsetting(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.PWMPINSETTING.val(), 16);
		//
		// M704 - PWMPINSETTING
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
