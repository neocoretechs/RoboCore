package com.neocoretechs.robocore.marlinspike.mcodes.status;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * 
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class motorcontrolSetting extends AbstractBasicDataLoader {
	private boolean DEBUG = false;
	public motorcontrolSetting(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.MOTORCONTROLSETTING.val(), 16);
		//
		// MOTORCONTROLSETTING
		//
	}
		@Override
		public Object getMachineReadingResult(MachineReading mr) {
			return mr;
		}
		@Override
		public MachineReading formatMachineReading(String sdata) {
			return new MachineReading(sdata);
		}
}
