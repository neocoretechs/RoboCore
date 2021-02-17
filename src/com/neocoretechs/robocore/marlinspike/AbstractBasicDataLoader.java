package com.neocoretechs.robocore.marlinspike;

import java.util.ArrayList;
import java.util.concurrent.BrokenBarrierException;

import com.neocoretechs.robocore.machine.bridge.MachineBridge;
import com.neocoretechs.robocore.machine.bridge.MachineReading;

/**
 * Abstraction of Marlinspike topic result handler that loads the data without any
 * special manipulation or consideration of special formats, merely line by line copy.
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public abstract class AbstractBasicDataLoader extends AbstractBasicResponse {
	private boolean DEBUG = false;
	int queueSize;
	public AbstractBasicDataLoader(AsynchDemuxer asynchDemuxer, String topicName, int queueSize) {
		super(asynchDemuxer, topicName, queueSize);
		this.queueSize = queueSize;
	}
	/**
	 * Typically just returning the passed MachineReading as the String payload therein,
	 * but may return the entire MachineReading if manipulation of the various sequence elements is necessary.
	 * This is used at the endpoint to deliver the final formatted MachineReadings from the MachineBridge
	 * to ultimate consumer.
	 * @param mr
	 * @return
	 */
	public abstract Object getMachineReadingResult(MachineReading mr); //return mr.getReadingValString();
	/**
	 * Format each element of the MachineReading to be added to MachineBridge from retrieved Marlinspike response lines.
	 * @param sdata The line by line Marlinspike responses
	 * @return the MachineReading formatted to spec
	 */
	public abstract MachineReading formatMachineReading(String sdata);
	
	@Override
	public void run(ArrayList<String> readLine) {
			MachineBridge mb = topicList.getMachineBridge();
			MachineReading mr = null;
			synchronized(mb) {
				for(String data: readLine) {
					if(data != null && data.length() > 0) {
						if(DEBUG)
							System.out.println(this.getClass().getName()+" "+topicName+" machineBridge:"+mb+" data:"+data);
						String sload = asynchDemuxer.parseDirective(data);
						boolean isTopic = (sload != null && sload.equals(topicName));
						if(sload != null && sload.length() > 0 && (asynchDemuxer.isLineTerminal(data) && isTopic)) 
							break;
						if( !isTopic ) {
							mr = formatMachineReading(data);
							mb.add(mr);
						}
					}
				}
				if(DEBUG)
					System.out.println(this.getClass().getName()+" "+topicName+" machineBridge:"+mb);
				//mb.add(MachineReading.EMPTYREADING);
				mb.notifyAll();
			}
			try {
				asynchDemuxer.mutexWrite.await();
			} catch(IllegalMonitorStateException | InterruptedException | BrokenBarrierException ims) {
					System.out.println(this.getClass().getName()+" "+Thread.currentThread().getName()+" "+
							ims+" "+topicName);
					ims.printStackTrace();
			} 
	}
}
