package com.neocoretechs.robocore.machine.bridge;

import com.neocoretechs.robocore.ThreadPoolManager;

/**
 * Retrieve the real time digital pin state from attached microcontroller and place in 2 element array
 * of pin, value.
 * This class functions with a series of singletons for each type of message coming form the real time
 * environment. For each 'listener' such as this, the getInstance method will start a thread that
 * activates the loop that takes elements of this group from the MachineBridge. These MachineBridge
 * readings are placed into the CircularBlockingDeque in this class for later retrieval.
 * <digitalpin>
 * 1 pin
 * 2 value
 * </digitalpin>
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
	private static volatile DigitalPinListener instance = null;
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
