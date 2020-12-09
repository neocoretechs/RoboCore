/**
 * 
 */
package com.neocoretechs.robocore.machine.bridge;

import com.neocoretechs.robocore.ThreadPoolManager;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * Motor control status returned as a string describing condition.
 * This class functions with a series of singletons for each type of message coming form the real time
 * environment. For each 'listener' such as this, the getInstance method will start a thread that
 * activates the loop that takes elements of this group from the MachineBridge. These MachineBridge
 * readings are placed into the CircularBlockingDeque in this class for later retrieval.
 * <MotorFault status channel power>
 * @author jg
 *
 */
public class MotorFaultListener implements Runnable {
	public static CircularBlockingDeque<String> data = new CircularBlockingDeque<String>(16);
	private static MachineBridge bridge;
	private static volatile MotorFaultListener instance = null;
	public static MotorFaultListener getInstance() {
		if( instance == null ) {
			synchronized(MotorFaultListener.class) { 
				if(instance == null) {
					instance = new MotorFaultListener();
				}
			}
		}
		return instance;
	}
	public MotorFaultListener() {
		ThreadPoolManager.getInstance().spin(this, AsynchDemuxer.topicNames.MOTORFAULT.val());
		bridge = MachineBridge.getInstance(AsynchDemuxer.topicNames.MOTORFAULT.val());
	}
	/* (non-Javadoc)
	 * @see java.lang.Runnable#run()
	 */
	@Override
	public void run() {
		while(true) {
				try {
					// put everything on the publish queue
					/*
					List<MachineReading> lim = MachineBridge.getInstance("motorfault").get();
					synchronized(lim) {
						for(int i = 0; i < lim.size(); i++) {
							if( lim.get(i) != null) {
								String sdata = lim.get(i).getReadingValString();
								if( sdata != null )
									data.add(sdata);
							}
						}
					}
					*/
					MachineReading mr = bridge.take();
					if( mr != null ) {
						String sdata = mr.getReadingValString();
						System.out.println("MOTOR FAULT:"+sdata);
						if( sdata != null )
							data.addLast(sdata);
					}
				} catch(IndexOutOfBoundsException ioobe) {}
		}
	}

}
