package com.neocoretechs.robocore.machine.bridge;

import com.neocoretechs.robocore.ThreadPoolManager;

/**
 * @author jg
 *
 */
public class DigitalPinListener implements Runnable {
	public static boolean DEBUG = false;
	public static CircularBlockingDeque<MachineReading> data = new CircularBlockingDeque<MachineReading>(16);
	private static DigitalPinListener instance = null;
	public static DigitalPinListener getInstance() { 
		if(instance == null)
			instance = new DigitalPinListener();
		return instance;		
	}
	private DigitalPinListener() {
		ThreadPoolManager.getInstance().spin(this, "digitalpin");
	}
	/* (non-Javadoc)
	 * @see java.lang.Runnable#run()
	 */
	@Override
	public void run() {
		while(true) {
			ThreadPoolManager.getInstance().waitGroup("digitalpin");
			try {
					MachineReading mr = MachineBridge.getInstance("digitalpin").take();
					if( mr != null ) {
						if( DEBUG )
							System.out.println(mr);
						data.addLast(mr);
					}
			} catch(IndexOutOfBoundsException ioobe) {}
			//try {
			//System.out.println("UltrasonicListener:"+MachineBridge.getInstance("ultrasonic").get(0)+" elems:"+data.size());
			//} catch(IndexOutOfBoundsException ioobe) {}
		}
	}

}
