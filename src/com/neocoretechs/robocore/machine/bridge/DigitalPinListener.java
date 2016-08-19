package com.neocoretechs.robocore.machine.bridge;

import com.neocoretechs.robocore.ThreadPoolManager;

/**
 * Retrieve the real time digital pin state from attached microcontroller and place in 2 element array
 * of pin, value
 * @author jg
 *
 */
public class DigitalPinListener implements Runnable {
	public static boolean DEBUG = true;
	public static int rightPivotPin = 46;
	public static int leftPivotPin = 47;
	public static int stopPin = 49;
	public static CircularBlockingDeque<int[]> data = new CircularBlockingDeque<int[]>(4);
	private static MachineBridge bridge;
	private static DigitalPinListener instance = null;
	public static DigitalPinListener getInstance() { 
		if(instance == null)
			instance = new DigitalPinListener();
		return instance;		
	}
	private DigitalPinListener() {
		ThreadPoolManager.getInstance().spin(this, "digitalpin");
		bridge = MachineBridge.getInstance("digitalpin");
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
						data.addLast(new int[]{ mr.getRawSeq(), mr.getReadingValInt() });
					}
			} catch(IndexOutOfBoundsException ioobe) {}
			//try {
			//System.out.println("UltrasonicListener:"+MachineBridge.getInstance("ultrasonic").get(0)+" elems:"+data.size());
			//} catch(IndexOutOfBoundsException ioobe) {}
		}
	}

}
