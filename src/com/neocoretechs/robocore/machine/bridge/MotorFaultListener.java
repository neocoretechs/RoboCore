/**
 * 
 */
package com.neocoretechs.robocore.machine.bridge;

import com.neocoretechs.robocore.ThreadPoolManager;
/**
 * Motor control status returned as a string describing condition
 * @author jg
 *
 */
public class MotorFaultListener implements Runnable {
	public static CircularBlockingDeque<String> data = new CircularBlockingDeque<String>(16);
	private static MotorFaultListener instance = null;
	public static MotorFaultListener getInstance() {
		if( instance == null )
			instance = new MotorFaultListener();
		return instance;
	}
	public MotorFaultListener() {
		ThreadPoolManager.getInstance().spin(this, "motorfault");
	}
	/* (non-Javadoc)
	 * @see java.lang.Runnable#run()
	 */
	@Override
	public void run() {
		while(true) {
			ThreadPoolManager.getInstance().waitGroup("motorfault");
				try {
					// put everything on the publish queue
					/*
					List<MachineReading> lim = MachineBridge.getInstance("motorfault").get();
					synchronized(lim) {
						for(int i = 0; i < lim.size(); i++) {
							if( lim.get(i) != null) {
								String sdata = lim.get(i).getReadingValString();
								if( sdata != null )
									data.add(sdata);
							}
						}
					}
					*/
					MachineReading mr = MachineBridge.getInstance("motorfault").take();
					if( mr != null ) {
						String sdata = mr.getReadingValString();
						if( sdata != null )
							data.addLast(sdata);
					}
				} catch(IndexOutOfBoundsException ioobe) {}
		}
	}

}
