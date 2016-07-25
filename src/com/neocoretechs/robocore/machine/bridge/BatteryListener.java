package com.neocoretechs.robocore.machine.bridge;

import com.neocoretechs.robocore.ThreadPoolManager;

/**
 * @author jg
 *
 */
public class BatteryListener implements Runnable {
	public static CircularBlockingDeque<Float> data = new CircularBlockingDeque<Float>(16);
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
				try {
					MachineReading mr = MachineBridge.getInstance("battery").take();
					if( mr != null ) {
						data.addLast(new Float(((float)mr.getReadingValInt())/10.0));
					}
				} catch(IndexOutOfBoundsException ioobe) {}
			//try {
			//	System.out.println("BatteryListener:"+MachineBridge.getInstance("battery").get(0)+" elems:"+data.size());
			//} catch(IndexOutOfBoundsException ioobe) {}
		}
	}

}
