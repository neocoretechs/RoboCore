package com.neocoretechs.robocore.marlinspike.mcodes.status;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicDataLoader;
/**
 * Report PWM controller setting, part of response to M705
 * @author Jonathan Groff (C0 NeoCoreTechs 2020,2021
 *
 */
public class PWMcontrolsetting extends AbstractBasicDataLoader {
	private boolean DEBUG = false;

	public PWMcontrolsetting(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.PWMCONTROLSETTING.val(), 16);
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
