package com.neocoretechs.robocore.marlinspike.mcodes.status;

import java.util.HashMap;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.MachineReading;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
import com.neocoretechs.robocore.marlinspike.AbstractBasicDataLoader;

/**
 * 2 = HBridge driver enable pin not found
 * 4 = SplitBridge driver enable pin not found 
 * 6 = SwitchBridge driver enable pin not found
 * 7 = Variable PWM driver enable pin not found
 * 1 = Set Motor Shutdown / Set PWM Shutdown
 * 0 = Set Motor run / Set PWM run
 * 8 = Ultrasonic shutdown
 * 10 = Encoder shutdown
 * 81 = M81
 * 799 = M799 Shutdown motor/PWM
 * -1 = M799 shutdown ALL motor/PWM
 * -2 = Kill method called
 * -3 = Stop method called
 * <Bad Motor command s c p/> status, channel, power
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class badmotor extends AbstractBasicDataLoader {
	private boolean DEBUG = false;
	private HashMap<String, String> faultCodes = new HashMap<String, String>();
	public badmotor(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer,topicNames.BADMOTOR.val(), 8);
		faultCodes.put("2","HBridge driver enable pin not found");
		faultCodes.put("4", "SplitBridge driver enable pin not found");
		faultCodes.put("6", "SwitchBridge driver enable pin not found");
		faultCodes.put("7", "Variable PWM driver enable pin not found");
		faultCodes.put("1", "Set Motor Shutdown / Set PWM Shutdown");
		faultCodes.put("0", "Set Motor run / Set PWM run");
		faultCodes.put("8", "Ultrasonic shutdown");
		faultCodes.put("10", "Encoder shutdown");
		faultCodes.put("81", "M81");
		faultCodes.put("799", "M799 Shutdown motor/PWM");
		faultCodes.put("-1", "M799 shutdown ALL motor/PWM");
		faultCodes.put("-2", "Kill method called");
		faultCodes.put("-3", "Stop method called");
		//
		// BADMOTOR
		//
	}
	
	@Override
	public Object getMachineReadingResult(MachineReading mr) {
		return mr.getReadingValString();
	}
	@Override
	public MachineReading formatMachineReading(String sdata) {
		if(asynchDemuxer.isLineTerminal(sdata)) {
			String sload = asynchDemuxer.extractPayload(sdata, topicNames.BADMOTOR.val());
			if(sload != null) {
				String[] sarray = sload.trim().split(" ");
				StringBuilder sout = new StringBuilder();
				if(sarray.length > 0) 
					sout.append(faultCodes.get(sarray[0])); 
				else
					sout.append("FAULT");
				sout.append(" channel ");
				if(sarray.length > 1)
					sout.append(sarray[1]);
				else
					sout.append("UNKNOWN");
				sout.append(" power ");
				if(sarray.length > 2)
					sout.append(sarray[2]);
				else
					sout.append("UNKNOWN");
				return new MachineReading(sout.toString());
			}
		}
		return new MachineReading(sdata);
	}
}
