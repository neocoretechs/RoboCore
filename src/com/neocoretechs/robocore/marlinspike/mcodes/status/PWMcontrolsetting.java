package com.neocoretechs.robocore.marlinspike.mcodes.status;


import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.marlinspike.AbstractBasicDataLoader;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * Report PWM controller setting, part of response to M705
 * @author Jonathan Groff (C0 NeoCoreTechs 2020,2021
 *
 */
public class PWMcontrolsetting extends AbstractBasicDataLoader {
	private boolean DEBUG = false;

	public PWMcontrolsetting(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer,  topicNames.PWMCONTROLSETTING.val(), 16);
		//
		// M705 -  PWMCONTROLSETTING
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
