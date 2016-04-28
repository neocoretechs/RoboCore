/**
 * 
 */
package com.neocoretechs.robocore.machine.bridge;

import java.util.concurrent.ArrayBlockingQueue;
import com.neocoretechs.robocore.ThreadPoolManager;

/**
 * @author jg
 *
 */
public class UltrasonicListener implements Runnable {
	public static boolean DEBUG = true;
	public static ArrayBlockingQueue<Integer> data = new ArrayBlockingQueue<Integer>(1024);
	public static int deleteThreshold = 1024; // number of readings before clear
	private static UltrasonicListener instance = null;
	public static UltrasonicListener getInstance() { 
		if(instance == null)
			instance = new UltrasonicListener();
		return instance;
			
	}
	private UltrasonicListener() {
		ThreadPoolManager.getInstance().spin(this, "ultrasonic");
	}
	/* (non-Javadoc)
	 * @see java.lang.Runnable#run()
	 */
	@Override
	public void run() {
		while(true) {
			ThreadPoolManager.getInstance().waitGroup("ultrasonic");
			if( data.size() > deleteThreshold/2 ) {
					for(int i = 0; i < deleteThreshold/2; i++)
						if( data.size() > 0) data.remove();
			}
			try {
					MachineReading mr = MachineBridge.getInstance("ultrasonic").take();
					if( mr != null ) {
						if( DEBUG )
							System.out.println(mr);
						data.add(new Integer(mr.getReadingValInt()));
					}
			} catch(IndexOutOfBoundsException ioobe) {}
			//try {
			//System.out.println("UltrasonicListener:"+MachineBridge.getInstance("ultrasonic").get(0)+" elems:"+data.size());
			//} catch(IndexOutOfBoundsException ioobe) {}
		}
	}

}
