package com.neocoretechs.robocore.machine.bridge;

import com.neocoretechs.robocore.ThreadPoolManager;

/**
 * @author jg
 *
 */
public class AnalogPinListener implements Runnable {
	public static boolean DEBUG = true;
	public static CircularBlockingDeque<MachineReading> data = new CircularBlockingDeque<MachineReading>(16);
	private static AnalogPinListener instance = null;
	
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
	}
	/* (non-Javadoc)
	 * @see java.lang.Runnable#run()
	 */
	@Override
	public void run() {
		while(true) {
			ThreadPoolManager.getInstance().waitGroup("analogpin");
			try {
					MachineReading mr = MachineBridge.getInstance("analogpin").take();
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
						data.addLast(mr);
						
					}
			} catch(IndexOutOfBoundsException ioobe) {}

		}
	}

}
