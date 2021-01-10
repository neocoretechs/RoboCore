package com.neocoretechs.robocore.marlinspike.mcodes.status;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * M44 P<pin> [U] - -Read digital pin with optional pullup
 * 		SERIAL_PGM(MSG_BEGIN);
 *			SERIAL_PGM(digitalPinHdr);
 *			SERIAL_PGMLN(MSG_DELIMIT);
 *			SERIAL_PGM("1 ");
 *			SERIAL_PORT.println(pin_number);
 *			SERIAL_PGM("2 ");
 *			SERIAL_PORT.println(res);
 *			SERIAL_PGM(MSG_BEGIN);
 *			SERIAL_PGM(digitalPinHdr);
 *			SERIAL_PGMLN(MSG_TERMINATE);
 *			SERIAL_PORT.flush();
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class digitalpin extends AbstractBasicDataLoader {
	private boolean DEBUG;
	public digitalpin(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.DIGITALPIN.val(), 5);
		//
		// M44
		//
	}

	@Override
	public Object getMachineReadingResult(MachineReading mr) {
		return mr;
	}
	@Override
	public MachineReading formatMachineReading(String sdata) {
		int reading = asynchDemuxer.getReadingNumber(sdata);
		int data =  asynchDemuxer.getReadingValueInt(sdata);
		return new MachineReading(1, reading, reading, data);
	}

}
