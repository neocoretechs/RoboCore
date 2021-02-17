package com.neocoretechs.robocore.marlinspike.mcodes.status;


import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.marlinspike.AbstractBasicDataLoader;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * Part of response to M705
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class motorcontrolSetting extends AbstractBasicDataLoader {
	private boolean DEBUG = false;
	public motorcontrolSetting(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer,  topicNames.MOTORCONTROLSETTING.val(), 16);
		//
		// M705 - MOTORCONTROLSETTING
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
