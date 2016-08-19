package com.neocoretechs.robocore.machine.bridge;

import com.neocoretechs.robocore.ThreadPoolManager;

/**
 * Retrieve the real time analog pin data from attached microcontroller and place in 2 element array
 * of pin, value
 * @author jg
 *
 */
public class AnalogPinListener implements Runnable {
	public static boolean DEBUG = false;
	public static CircularBlockingDeque<int[]> data = new CircularBlockingDeque<int[]>(4);
	private static AnalogPinListener instance = null;
	private static MachineBridge bridge;
	
	// analog inputs on pins 55,56 of Mega2560 as defined in startup.gcode for AsynchDemuxer
	public final static int joystickPinY = 55;
	public final static int joystickPinX = 56;
	
	private int yDeadMin = 500;
	private int yDeadMax = 510;
	private int xDeadMin = 670;
	private int xDeadMax = 700;
	public static AnalogPinListener getInstance() { 
		if(instance == null)
			instance = new AnalogPinListener();
		return instance;		
	}
	private AnalogPinListener() {
		ThreadPoolManager.getInstance().spin(this, "analogpin");
		bridge = MachineBridge.getInstance("analogpin");
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
						if( mr.getRawSeq() == joystickPinY) {// linear pin
							if(mr.getReadingValInt() >= yDeadMin && mr.getReadingValInt() <= yDeadMax )
								continue;
						} else {
							if( mr.getRawSeq() == joystickPinX) {// ang pin
								if(mr.getReadingValInt() >= xDeadMin && mr.getReadingValInt() <= xDeadMax )
									continue;
							}
						}
						data.addLast(new int[]{ mr.getRawSeq(), mr.getReadingValInt() });	
					}
			} catch(IndexOutOfBoundsException ioobe) {}

		}
	}

}
