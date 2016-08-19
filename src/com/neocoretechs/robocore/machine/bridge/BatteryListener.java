package com.neocoretechs.robocore.machine.bridge;

import com.neocoretechs.robocore.ThreadPoolManager;

/**
 * @author jg
 *
 */
public class BatteryListener implements Runnable {
	public static CircularBlockingDeque<Float> data = new CircularBlockingDeque<Float>(16);
	private static BatteryListener instance = null;
	private static MachineBridge bridge;
	public static BatteryListener getInstance() {
		if( instance == null ) {
			instance = new BatteryListener();
		}
		return instance;
	}
	private BatteryListener() {
		ThreadPoolManager.getInstance().spin(this, "battery");
		bridge = MachineBridge.getInstance("battery");
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
						data.addLast(new Float(((float)mr.getReadingValInt())/10.0));
					}
				} catch(IndexOutOfBoundsException ioobe) {}
			//try {
			//	System.out.println("BatteryListener:"+MachineBridge.getInstance("battery").get(0)+" elems:"+data.size());
			//} catch(IndexOutOfBoundsException ioobe) {}
		}
	}

}
