package com.neocoretechs.robocore.marlinspike.mcodes.status;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicDataLoader;

/**
 * M46 P<pin> - -Read analog pin 
 *	SERIAL_PGM(MSG_BEGIN);
 *	SERIAL_PGM(analogPinHdr);
 *	SERIAL_PGMLN(MSG_DELIMIT);
 *	SERIAL_PGM("1 ");
 *	SERIAL_PORT.println(pin_number);
 *	SERIAL_PGM("2 ");
 *	SERIAL_PORT.println(res);
 *	SERIAL_PGM(MSG_BEGIN);
 *	SERIAL_PGM(analogPinHdr);
 *	SERIAL_PGMLN(MSG_TERMINATE);
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class analogpin extends AbstractBasicDataLoader {
	private boolean DEBUG;
	public analogpin(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.ANALOGPIN.val(), 5);
	}
	
	@Override
	public Object getMachineReadingResult(MachineReading mr) {
		return mr;
	}
	@Override
	public MachineReading formatMachineReading(String sdata) {
		int reading = asynchDemuxer.getReadingNumber(sdata);
		int datai =  asynchDemuxer.getReadingValueInt(sdata);
		return new MachineReading(1, reading, reading, datai);
	}

}
