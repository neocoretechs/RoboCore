package com.neocoretechs.robocore.marlinspike.mcodes.status;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
* 	SERIAL_PGM(MSG_BEGIN);
*	SERIAL_PGM(motorFaultCntrlHdr);
*	SERIAL_PGMLN(MSG_DELIMIT);
*	for(int i = 0; i < 8 ; i++) {
*		bfault = fault & (1<<i);
*		switch(bfault) {
*			default:
*			case 0: // bit not set
*				break;
*			case 1:
*				SERIAL_PORT.print(j++);
*				SERIAL_PORT.print(' ');
*				SERIAL_PGMLN(MSG_MOTORCONTROL_1);
*				break;
*			case 2:
*				SERIAL_PORT.print(j++);
*				SERIAL_PORT.print(' ');
*				SERIAL_PGMLN(MSG_MOTORCONTROL_2);
*				break;
*			case 4:
*				SERIAL_PORT.print(j++);
*				SERIAL_PORT.print(' ');
*				SERIAL_PGMLN(MSG_MOTORCONTROL_3);
*				break;
*			case 8:
*				SERIAL_PORT.print(j++);
*				SERIAL_PORT.print(' ');
*				SERIAL_PGMLN(MSG_MOTORCONTROL_4);
*				break;
*			case 16:
*				SERIAL_PORT.print(j++);
*				SERIAL_PORT.print(' ');
*				SERIAL_PGMLN(MSG_MOTORCONTROL_5);
*				break;
*			case 32:
*				SERIAL_PORT.print(j++);
*				SERIAL_PORT.print(' ');
*				SERIAL_PGMLN(MSG_MOTORCONTROL_6);
*				break;
*			case 64:
*				SERIAL_PORT.print(j++);
*				SERIAL_PORT.print(' ');
*				SERIAL_PGMLN(MSG_MOTORCONTROL_7);
*				break;
*			case 128:
*				SERIAL_PORT.print(j++);
*				SERIAL_PORT.print(' ');
*				SERIAL_PGMLN(MSG_MOTORCONTROL_8);
*				break;
*		}
*	}
*	SERIAL_PGM(MSG_BEGIN);
*	SERIAL_PGM(motorFaultCntrlHdr);
*	SERIAL_PGMLN(MSG_TERMINATE);
* f1 = overheat
* f2 = overvoltage
* f3 = undervoltage
* f4 = short circuit
* f5 = emergency stop
* f6 = Sepex excitation fault
* f7 = MOSFET failure
* f8 = startup configuration fault
* @author Jonathan Groff Copyright (C) NeoCoreTechs 2020,2021
*
*/
public class motorfault extends AbstractBasicDataLoader {
	private boolean DEBUG;

	public motorfault(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.MOTORFAULT.val(), 16);
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
