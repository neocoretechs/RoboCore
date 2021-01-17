package com.neocoretechs.robocore.marlinspike.mcodes.status;


import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicDataLoader;
/**
 * M503
 * 		SERIAL_PGM(MSG_BEGIN);
 *		SERIAL_PGM(eepromHdr);
 *		SERIAL_PGMLN(MSG_DELIMIT);
 *		SERIAL_PGM("1 "); // parameter 1
 *		SERIAL_PORT.println(param);
 *		SERIAL_PGM("2 "); // parameter 2
 *		SERIAL_PORT.println(param); // param
 *		SERIAL_PGM(MSG_BEGIN);
 *		SERIAL_PGM(eepromHdr);
 *		SERIAL_PGMLN(MSG_TERMINATE);
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class eeprom extends AbstractBasicDataLoader {
	private boolean DEBUG;
	public eeprom(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.EEPROM.val(), 16);
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
		String datax =  asynchDemuxer.getReadingValueString(sdata);
		return new MachineReading(1, reading, reading, datax);
	}

}
