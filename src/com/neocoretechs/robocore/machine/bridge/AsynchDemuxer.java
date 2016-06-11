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
	private static String[] topicNames = new String[]{"dataset","battery","motorfault","ultrasonic"};
	
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
				}
				//if( Props.DEBUG) System.out.println("Signal end dataset...");
				mb.signalEnd();
			}		
		});
		// listeners for dataset are in individual run instances
        MachineBridge.getInstance("battery").init();
		topics.put("battery",new TopicList() {
			@Override
			public void retrieveData() {
		        MachineBridge mb = MachineBridge.getInstance("battery");
			    //mb.init();
			    String readLine;
				while( (readLine = ByteSerialDataPort.getInstance().readLine()) != null ) {
						if( readLine.length() == 0 ) {
							//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
							continue;
						}
						int reading = AbstractMachine.getReadingNumber(readLine);
						int data =  AbstractMachine.getReadingValueInt(readLine);
						//if( Props.DEBUG ) System.out.println(readLine);
						MachineReading mr = new MachineReading(1, reading, reading+1, data);
						mb.add(mr);
				}
				//if( Props.DEBUG) System.out.println("Signal end battery...");
				mb.signalEnd();
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
				while( (readLine = ByteSerialDataPort.getInstance().readLine()) != null ) {
					if( readLine.length() == 0 ) {
						//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
						continue;
					}
					int reading = AbstractMachine.getReadingNumber(readLine);
					String data =  AbstractMachine.getReadingValueString(readLine);
					//if( Props.DEBUG ) System.out.println(readLine);
					MachineReading mr = new MachineReading(1, reading, reading+1, data);
					mb.add(mr);
				}
				//if( Props.DEBUG) System.out.println("Signal end motorfault...");
				mb.signalEnd();
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
				while( (readLine = ByteSerialDataPort.getInstance().readLine()) != null ) {
						if( readLine.length() == 0 ) {
							//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
							continue;
						}
						int reading = AbstractMachine.getReadingNumber(readLine);
						int data =  AbstractMachine.getReadingValueInt(readLine);
						if( DEBUG ) 
							System.out.println("Ultrasonic retrieveData:"+readLine+"| converted:"+reading+" "+data);
						MachineReading mr = new MachineReading(1, reading, reading+1, data);
						mb.add(mr);
				}
				if( DEBUG) 
					System.out.println("Signal end ultrasonic...");
				mb.signalEnd();
			}
		});
		// start an ultrasonic listener
		UltrasonicListener.getInstance();
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
				//System.out.println("op:"+op.toString());
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

}
