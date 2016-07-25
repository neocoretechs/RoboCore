/**
 * 
 */
package com.neocoretechs.robocore.machine.bridge;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import com.neocoretechs.robocore.ThreadPoolManager;
import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;

/**
 * @author jg
 *
 */
public class AsynchDemuxer implements Runnable {
	private static boolean DEBUG = true;
	private boolean shouldRun = true;
	private static AsynchDemuxer instance = null;
	private AsynchDemuxer() {}
	public static AsynchDemuxer getInstance() {
		if( instance == null ) {
			instance = new AsynchDemuxer();
			instance.init();
			ThreadPoolManager.getInstance().spin(instance, "SYSTEM");
		}
		return instance;
	}
	private Map<String, TopicList> topics = new HashMap<String, TopicList>();
	private static String[] topicNames = new String[]{"dataset","battery","motorfault","ultrasonic","digitalpin","analogpin"};
	
	public static String[] getTopicNames() { return topicNames; }
	
	private void init() {
		ThreadPoolManager.init(topicNames);
        MachineBridge.getInstance("dataset").init();
		topics.put("dataset", new TopicList() {
			@Override
			public void retrieveData() {
		        MachineBridge mb = MachineBridge.getInstance("dataset");
		        String readLine;
				while( (readLine = ByteSerialDataPort.getInstance().readLine()) != null ) {
					if( readLine.length() == 0 ) {
						//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
						continue;
					}
					int reading = AbstractMachine.getReadingNumber(readLine);
					double data =  AbstractMachine.getReadingValueDouble(readLine);
					//if( Props.DEBUG ) System.out.println(readLine);
					MachineReading mr = new MachineReading(1, reading, reading+1, data);
					mb.add(mr);
					ThreadPoolManager.getInstance().notifyGroup("dataset");
				}
			}		
		});
		// listeners for dataset are in individual run instances
        MachineBridge.getInstance("battery").init();
		topics.put("battery",new TopicList() {
			@Override
			public void retrieveData() {
		        MachineBridge mb = MachineBridge.getInstance("battery");
			    //mb.init();
			    String readLine = ByteSerialDataPort.getInstance().readLine();
				if( readLine == null || readLine.length() == 0 ) {
							//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
							return;
				}
				int reading = AbstractMachine.getReadingNumber(readLine);
				int data =  AbstractMachine.getReadingValueInt(readLine);
				//if( Props.DEBUG ) System.out.println(readLine);
				MachineReading mr = new MachineReading(1, reading, reading+1, data);
				mb.add(mr);
				ThreadPoolManager.getInstance().notifyGroup("battery");

			}
		});
		// start the listener thread for this topic
		BatteryListener.getInstance();
		
		MachineBridge.getInstance("motorfault").init();
		topics.put("motorfault", new TopicList() {
			@Override
			public void retrieveData() {
				MachineBridge mb = MachineBridge.getInstance("motorfault");
				//mb.init();
				String readLine;
				for(int i = 0; i < 8; i++) {
					readLine = ByteSerialDataPort.getInstance().readLine();
					if( readLine == null || readLine.length() == 0 ) {
						//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
						continue;
					}
					int reading = AbstractMachine.getReadingNumber(readLine);
					String data =  AbstractMachine.getReadingValueString(readLine);
					//if( Props.DEBUG ) System.out.println(readLine);
					MachineReading mr = new MachineReading(1, reading, reading+1, data);
					mb.add(mr);
					ThreadPoolManager.getInstance().notifyGroup("motorfault");
				}

			}
		});
		MotorFaultListener.getInstance();
		
		MachineBridge.getInstance("ultrasonic").init();
		topics.put("ultrasonic", new TopicList() {
			@Override
			public void retrieveData() {
				MachineBridge mb = MachineBridge.getInstance("ultrasonic");
				//mb.init();
				String readLine;
				readLine = ByteSerialDataPort.getInstance().readLine();
				if( readLine == null || readLine.length() == 0 ) {
							//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
						return;
				}
				int reading = AbstractMachine.getReadingNumber(readLine);
				int data =  AbstractMachine.getReadingValueInt(readLine);
				if( DEBUG ) 
						System.out.println("Ultrasonic retrieveData:"+readLine+"| converted:"+reading+" "+data);
				MachineReading mr = new MachineReading(1, reading, reading+1, data);
				mb.add(mr);
				if( DEBUG) 
					System.out.println("Signal end ultrasonic...");
				ThreadPoolManager.getInstance().notifyGroup("ultrasonic");
			}
		});
		// start an ultrasonic listener
		UltrasonicListener.getInstance();
		
		MachineBridge.getInstance("analogpin").init();
		topics.put("analogpin", new TopicList() {
			@Override
			public void retrieveData() {
				MachineBridge mb = MachineBridge.getInstance("analogpin");
				//mb.init();
				String readLine;
				int pin = 0;
				readLine = ByteSerialDataPort.getInstance().readLine();
				if( readLine == null || readLine.length() == 0 ) {
					return;
				}
				int reading = AbstractMachine.getReadingNumber(readLine);
				int data =  AbstractMachine.getReadingValueInt(readLine);
				if( DEBUG ) 
						System.out.println("analog pin retrieveData:"+readLine+"| converted:"+reading+" "+data);
				pin = data;
				readLine = ByteSerialDataPort.getInstance().readLine();
				if( readLine == null || readLine.length() == 0 ) {
					return;
				}
				reading = AbstractMachine.getReadingNumber(readLine);
				data =  AbstractMachine.getReadingValueInt(readLine);
				if( DEBUG ) 
					System.out.println("analog pin retrieveData:"+readLine+"| converted:"+reading+" "+data);
				MachineReading mr = new MachineReading(1, pin, reading, data);
				mb.add(mr);		
				if( DEBUG) 
					System.out.println("Signal end analogpin...");
				ThreadPoolManager.getInstance().notifyGroup("analogpin");
			}
		});
		// start an analog pin listener
		AnalogPinListener.getInstance();
		
		MachineBridge.getInstance("digitalpin").init();
		topics.put("digitalpin", new TopicList() {
			@Override
			public void retrieveData() {
				MachineBridge mb = MachineBridge.getInstance("digitalpin");
				//mb.init();
				String readLine;
				int pin = 0;
				readLine = ByteSerialDataPort.getInstance().readLine();
				if( readLine == null || readLine.length() == 0 ) {
					System.out.println("premature return digitalpin retrieveData");
					return;
				}
				int reading = AbstractMachine.getReadingNumber(readLine);
				int data =  AbstractMachine.getReadingValueInt(readLine);
				if( DEBUG ) 
						System.out.println("digital pin retrieveData:"+readLine+"| converted:"+reading+" "+data);
				pin = data;
				readLine = ByteSerialDataPort.getInstance().readLine();
				if( readLine == null || readLine.length() == 0 ) {
					System.out.println("premature return digitalpin retrieveData");
					return;
				}
				reading = AbstractMachine.getReadingNumber(readLine);
				data =  AbstractMachine.getReadingValueInt(readLine);
				if( DEBUG ) 
						System.out.println("digital pin retrieveData:"+readLine+"| converted:"+reading+" "+data);	
				MachineReading mr = new MachineReading(1, pin, reading, data);
				mb.add(mr);	
				
				if( DEBUG) 
					System.out.println("Signal end digitalpin...");
				ThreadPoolManager.getInstance().notifyGroup("digitalpin");
			}
		});
		// start a digital pin listener
		DigitalPinListener.getInstance();
	}
	
	/**
	 * Configure the robot with a series of G-code directives at startup in file startup.gcode
	 * @throws IOException
	 */
	public void config() throws IOException {
		// now read the startup G-code directives to initiate
		try {
			String[] starts = FileIOUtilities.readAllLines("", "startup.gcode", ";");
			for(String s : starts) {
				System.out.println("Startup GCode:"+s);
				ByteSerialDataPort.getInstance().writeLine(s+"\r");
			}
		} catch (IOException e) {
			if( DEBUG) System.out.println("No startup.gcode file detected..");
		}
	}
	/* (non-Javadoc)
	 * @see java.lang.Runnable#run()
	 */
	@Override
	public void run() {
		while(shouldRun) {
			StringBuffer op = new StringBuffer();
			int r;
			int i;
			try {
				if((i=ByteSerialDataPort.getInstance().read()) != '<' ) {
					if( i == -1)
						System.out.println("Looking for directive but found EOF");
					else
						System.out.println("Looking for directive but found "+i+" "+Character.toChars(i)[0]);
					continue;
				}
				while((r = ByteSerialDataPort.getInstance().read()) != '>') {
						op.append((char)r);
						if( op.length() > 128 ) {
							System.out.println("Directive exceeds legal length:"+op);
							op = new StringBuffer();
							continue;
						}
				}
				// Soak up c/r
				ByteSerialDataPort.getInstance().read();
				// soak up l/f
				ByteSerialDataPort.getInstance().read();
				if(DEBUG)
					System.out.println("op:"+op.toString());
				if( op.length() == 0 )
					continue;
			} catch (IOException ioe) {
				System.out.println("AsynchDemux IO exception:"+ioe);
				continue;
			}
			//if( Props.DEBUG ) System.out.println("Demuxing "+op.toString());
			TopicList tl = topics.get(op.toString());
			if( tl != null )
				tl.retrieveData();
			else
				System.out.println("Cannot demux received directive:"+op.toString());
			
		} // shouldRun	
	}
	
	private static interface TopicList {
		public void retrieveData();
	}
	
	public static void main(String[] args) throws Exception {
		// start demux
		AsynchDemuxer.getInstance();
		// the L H and T values represent those to EXCLUDE
		// So we are looking for state 0 on digital pin and value not between L and H analog
		ByteSerialDataPort.getInstance().writeLine("M303 P54 L470 H510");
		ByteSerialDataPort.getInstance().writeLine("M303 P55 L470 H510");
		ByteSerialDataPort.getInstance().writeLine("M305 P30 T1");
		ByteSerialDataPort.getInstance().writeLine("M305 P46 T1");
		ByteSerialDataPort.getInstance().writeLine("M305 P47 T1");
		ByteSerialDataPort.getInstance().writeLine("M305 P49 T1");
	}

}
