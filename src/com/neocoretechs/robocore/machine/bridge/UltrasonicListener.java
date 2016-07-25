package com.neocoretechs.robocore.machine.bridge;

import com.neocoretechs.robocore.ThreadPoolManager;

/**
 * @author jg
 *
 */
public class UltrasonicListener implements Runnable {
	public static boolean DEBUG = true;
	public static CircularBlockingDeque<Integer> data = new CircularBlockingDeque<Integer>(16);
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
			try {
					MachineReading mr = MachineBridge.getInstance("ultrasonic").take();
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
