package com.neocoretechs.robocore.marlinspike.mcodes.status;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicDataLoader;
/**
 * M47 -Read analog pin P<pin> T<threshold> compare to battery threshold, if below, print battery message.<p/>
 * Unlike other operations, this one will ack with either the return M47/, or a battery message:<br/>
 * 	SERIAL_PGM(MSG_BEGIN);<br/>
 *	SERIAL_PGM(batteryCntrlHdr);<br/>
 *	SERIAL_PGMLN(MSG_DELIMIT);<br/>
 *	SERIAL_PGM("1 ");<br/>
 *	SERIAL_PORT.println(volts);<br/>
 *	SERIAL_PGM(MSG_BEGIN);<br/>
 *	SERIAL_PGM(batteryCntrlHdr);<br/>
 *	SERIAL_PGMLN(MSG_TERMINATE);<br/>
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class battery extends AbstractBasicDataLoader {
	private boolean DEBUG;
	public battery(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.BATTERY.val(), 5);
		//
		// battery
		//
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
