package com.neocoretechs.robocore.marlinspike.mcodes.status;

import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.marlinspike.AbstractBasicDataLoader;
import com.neocoretechs.robocore.marlinspike.ActivationInterface;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.TypeSlotChannelEnable;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * M44 P<pin> [U] - -Read digital pin with optional pullup
 * 		SERIAL_PGM(MSG_BEGIN);
 *			SERIAL_PGM(digitalPinHdr);
 *			SERIAL_PGMLN(MSG_DELIMIT);
 *			SERIAL_PGM("0 ");
 *			SERIAL_PORT.println(pin_number);
 *			SERIAL_PGM("1 ");
 *			SERIAL_PORT.println(res);
 *			SERIAL_PGM(MSG_BEGIN);
 *			SERIAL_PGM(digitalPinHdr);
 *			SERIAL_PGMLN(MSG_TERMINATE);
 *			SERIAL_PORT.flush();
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
 *
 */
public class digitalpin extends AbstractBasicDataLoader implements ActivationInterface {
	private boolean DEBUG;
	private TypeSlotChannelEnable tsce; 
	public digitalpin(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.DIGITALPIN.val(), 5);
		//
		// M44
		//
	}
	
	public digitalpin(TypeSlotChannelEnable tsce) {
		this.tsce = tsce;
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

	@Override
	public String getActivation(int deviceLevel) {
		return String.format("M44 P%d%n", tsce.getPin());
	}

}
