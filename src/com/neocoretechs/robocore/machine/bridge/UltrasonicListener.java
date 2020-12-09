package com.neocoretechs.robocore.machine.bridge;

import com.neocoretechs.robocore.ThreadPoolManager;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;

/**
 * This class functions with a series of singletons for each type of message coming form the real time
 * environment. For each 'listener' such as this, the getInstance method will start a thread that
 * activates the loop that takes elements of this group from the MachineBridge. These MachineBridge
 * readings are placed into the CircularBlockingDeque in this class for later retrieval.
 * Here we are retrieving pin number as element 1 and reading as element 2 of dataset.
 * @author jg
 *
 */
public class UltrasonicListener implements Runnable {
	public static boolean DEBUG = false;
	public static CircularBlockingDeque<Integer> data = new CircularBlockingDeque<Integer>(16);
	private MachineBridge bridge;
	private static volatile UltrasonicListener instance = null;
	public static UltrasonicListener getInstance() { 
		if(instance == null) {
			synchronized(UltrasonicListener.class) { 
				instance = new UltrasonicListener();
			}
		}
		return instance;		
	}

	private UltrasonicListener() {
		ThreadPoolManager.getInstance().spin(this, topicNames.ULTRASONIC.val());
		bridge = MachineBridge.getInstance(topicNames.ULTRASONIC.val());
	}
	/* (non-Javadoc)
	 * @see java.lang.Runnable#run()
	 */
	@Override
	public void run() {
		while(true) {
			try {
					MachineReading mr = bridge.take();
					if( mr != null ) {
						if( DEBUG )
							System.out.println(mr);
						data.addLast(new Integer(mr.getReadingValInt()));
					}
			} catch(IndexOutOfBoundsException ioobe) {}
			//try {
			//System.out.println("UltrasonicListener:"+MachineBridge.getInstance("ultrasonic").get(0)+" elems:"+data.size());
			//} catch(IndexOutOfBoundsException ioobe) {}
		}
	}

}
