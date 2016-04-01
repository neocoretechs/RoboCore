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
public class BatteryListener implements Runnable {
	public static ArrayBlockingQueue<Float> data = new ArrayBlockingQueue<Float>(1024);
	public static int deleteThreshold = 10; // number of readings before clear
	private static BatteryListener instance = null;
	public static BatteryListener getInstance() {
		if( instance == null ) {
			instance = new BatteryListener();
		}
		return instance;
	}
	private BatteryListener() {
		ThreadPoolManager.getInstance().spin(this, "battery");
	}
	/* (non-Javadoc)
	 * @see java.lang.Runnable#run()
	 */
	@Override
	public void run() {
		while(true) {
			ThreadPoolManager.getInstance().waitGroup("battery");
				if( data.size() > deleteThreshold/2 ) {
					for(int i = 0; i < deleteThreshold/2; i++)
						if( data.size() > 0 ) data.remove();
				}
				try {
					MachineReading mr = MachineBridge.getInstance("battery").take();
					if( mr != null ) {
						data.add(new Float(((float)mr.getReadingValInt())/10.0));
					}
				} catch(IndexOutOfBoundsException ioobe) {}
			//try {
			//	System.out.println("BatteryListener:"+MachineBridge.getInstance("battery").get(0)+" elems:"+data.size());
			//} catch(IndexOutOfBoundsException ioobe) {}
		}
	}

}
