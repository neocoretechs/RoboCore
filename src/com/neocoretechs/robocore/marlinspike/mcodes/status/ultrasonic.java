package com.neocoretechs.robocore.marlinspike.mcodes.status;

import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.marlinspike.AbstractBasicDataLoader;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * M300
 * 		SERIAL_PGM(MSG_BEGIN);
 *		SERIAL_PGM(sonicCntrlHdr);
 *		SERIAL_PGMLN(MSG_DELIMIT);
 *		SERIAL_PGM("1 "); // pin
 *		SERIAL_PORT.println(pin_number);
 *		SERIAL_PGM("2 "); // sequence
 *		SERIAL_PORT.println(upin->getRange()); // range
 *		SERIAL_PGM(MSG_BEGIN);
 *		SERIAL_PGM(sonicCntrlHdr);
 *		SERIAL_PGMLN(MSG_TERMINATE);
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class ultrasonic extends AbstractBasicDataLoader {
	private boolean DEBUG;

	public ultrasonic(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer,  topicNames.ULTRASONIC.val(), 8);
		//
		// M46
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
