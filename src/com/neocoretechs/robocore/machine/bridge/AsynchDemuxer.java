package com.neocoretechs.robocore.machine.bridge;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import com.neocoretechs.robocore.ThreadPoolManager;
import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;

/**
 * This class is the primary interface between real time data and the other subsystems.
 * Its primary function is to demultiplex the input stream coming from data sources such as the attached
 * microcontroller Mega2560 etc that utilize a protocol with '<header>',line number, data value.
 * Each 'topic' described by the demultiplexed header as it flows in with its associated data is expected to have:
 * OPTIONAL:
 * 1) A thread that services the final processing listener class and deque for the given topic '<header>'
 * 2) A listener class that is serviced by the above thread that takes raw MachineReadings and transforms them if necessary
 * MANDATORY:
 * 3) A 'TopicList' class in the hash table with its associated 'header'
 * 4) An instance of 'MachineBridge' that operates on the raw data for a given topic 'header'
 * The optional items are necessary for data streamed at high rates. Notice in the code that 'dataset' has no
 * associated listener since it is a low volume data item. In this case the item is retrieved from the MachineBridge deque itself
 * rather than the associated listener deque.
 * As the various topics are demuxxed by the thread servicing this class, the 'retrieveData' for each 'TopicList' 
 * is invoked to place the 'MachineReading' element in the deque associated with the MachineBridge for that topic.
 * The listener waits for a take from that particular MachineBridge and massages the data to be placed in its own deque
 * in the format and with the proper additions and exclusions from the raw MachineReading.
 * When an element in the listener is present and ready for a 'take' from that deque the item is considered ready for use
 * in the system.
 * The size of each listener circular deque is determined during invocation of the 'init' method of the MachineBridge for that topic.
 * This demuxxer runs in its own thread as well such that it may operate unimpeded while the listeners can take their time
 * processing the data. In this way a near realtime response is preserved.
 * @author jg
 *
 */
public class AsynchDemuxer implements Runnable {
	private static boolean DEBUG = false;
	private boolean shouldRun = true;
	private boolean isRunning = false;
	private static AsynchDemuxer instance = null;
	private AsynchDemuxer() {}
	public static AsynchDemuxer getInstance() {
		if( instance == null ) {
			instance = new AsynchDemuxer();
			instance.init();
			ThreadPoolManager.getInstance().spin(instance, "SYSTEM");
			while(!instance.isRunning)
				try {
					Thread .sleep(1);
				} catch (InterruptedException e) {}
		}
		return instance;
	}
	private Map<String, TopicList> topics = new HashMap<String, TopicList>();
	private static String[] topicNames = new String[]{"dataset","battery","motorfault","ultrasonic","digitalpin","analogpin"};
	
	public static String[] getTopicNames() { return topicNames; }
	
	
	private void init() {
		ThreadPoolManager.init(topicNames);
		
        MachineBridge.getInstance("dataset").init(16);
		topics.put("dataset", new TopicList() {
			MachineBridge mb = MachineBridge.getInstance("dataset");
			@Override
			public void retrieveData() {  
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
			}		
		});

        MachineBridge.getInstance("battery").init(16);
		topics.put("battery",new TopicList() {
	        MachineBridge mb = MachineBridge.getInstance("battery");
			@Override
			public void retrieveData() {
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
			}
		});
		// start the listener thread for this topic
		BatteryListener.getInstance();
		
		MachineBridge.getInstance("motorfault").init(16);
		topics.put("motorfault", new TopicList() {
			MachineBridge mb = MachineBridge.getInstance("motorfault");
			@Override
			public void retrieveData() {
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
				}
			}
		});
		MotorFaultListener.getInstance();
		
		MachineBridge.getInstance("ultrasonic").init(16);
		topics.put("ultrasonic", new TopicList() {
			MachineBridge mb = MachineBridge.getInstance("ultrasonic");
			@Override
			public void retrieveData() {	
				//mb.init();
				String readLine;
				int pin = 0;
				readLine = ByteSerialDataPort.getInstance().readLine();
				if( readLine == null || readLine.length() == 0 ) {
							//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
						return;
				}
				int reading = AbstractMachine.getReadingNumber(readLine);
				int data =  AbstractMachine.getReadingValueInt(readLine);
				pin = data;
				readLine = ByteSerialDataPort.getInstance().readLine();
				if( readLine == null || readLine.length() == 0 ) {
							//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
						return;
				}
				reading = AbstractMachine.getReadingNumber(readLine);
				data =  (int) AbstractMachine.getReadingValueDouble(readLine);
				if( DEBUG ) 
						System.out.println("Ultrasonic retrieveData pin:"+pin+"| converted:"+reading+" "+data);
				MachineReading mr = new MachineReading(1, pin, reading, data);
				mb.add(mr);
			}
		});
		// start an ultrasonic listener
		UltrasonicListener.getInstance();
		
		MachineBridge.getInstance("analogpin").init(16);
		topics.put("analogpin", new TopicList() {
			MachineBridge mb = MachineBridge.getInstance("analogpin");
			@Override
			public void retrieveData() {
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
			}
		});
		// start an analog pin listener
		AnalogPinListener.getInstance();
		
		MachineBridge.getInstance("digitalpin").init(16);
		topics.put("digitalpin", new TopicList() {
			MachineBridge mb = MachineBridge.getInstance("digitalpin");
			@Override
			public void retrieveData() {
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
				//if( DEBUG ) 
						System.out.println("digital pin retrieveData:"+readLine+"| converted:"+reading+" "+data);	
				MachineReading mr = new MachineReading(1, pin, reading, data);
				mb.add(mr);	
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
			ByteSerialDataPort bsdp = ByteSerialDataPort.getInstance();
			String[] starts = FileIOUtilities.readAllLines("", "startup.gcode", ";");
			for(String s : starts) {
				System.out.println("Startup GCode:"+s);
				bsdp.writeLine(s+"\r");
				Thread.sleep(10);
			}
		} catch (IOException e) {
			if( DEBUG) System.out.println("No startup.gcode file detected..");
		} catch (InterruptedException e) {}
	}
	/* (non-Javadoc)
	 * @see java.lang.Runnable#run()
	 */
	@Override
	public void run() {
		isRunning = true;
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
		isRunning = false;
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
