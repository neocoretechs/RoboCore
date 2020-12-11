package com.neocoretechs.robocore.machine.bridge;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

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
	private static boolean DEBUG = true;
	private volatile boolean shouldRun = true;
	private volatile boolean isRunning = false;
	private volatile static AsynchDemuxer instance = null;

	public static enum topicNames {
		DATASET("dataset"),
		BATTERY("battery"),
		MOTORFAULT("motorfault"),
		ULTRASONIC("ultrasonic"),
		DIGITALPIN("digitalpin"),
		ANALOGPIN("analogpin"),
		ASSIGNEDPINS("assignedpins"),
		MOTORCONTROLSETTING("motorcontrolsetting"),
		PWMCONTROLSETTING("pwmcontrolsetting"),
		CONTROLLERSTATUS("controllerstatus");
		String name;
		topicNames(String name) { this.name = name;}
		public String val() { return name; }
	};
	
	private AsynchDemuxer() {}
	/**
	 * Double check lock singleton
	 * @return
	 */
	public static AsynchDemuxer getInstance() {
		if(instance == null ) {
			synchronized(AsynchDemuxer.class) {	
				if( instance == null ) {
					instance = new AsynchDemuxer();
				}
			}
		}
		return instance;
	}
	private Map<String, TopicList> topics = new ConcurrentHashMap<String, TopicList>();

	public void connect() throws IOException {
		ByteSerialDataPort.getInstance().connect(true);
	}
	
	public synchronized void init() {
		topicNames[] xtopics = topicNames.values();
		String[] stopics = new String[xtopics.length];
		for(int i = 0; i < xtopics.length; i++) stopics[i] = xtopics[i].val();
		ThreadPoolManager.init(stopics);
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.DATASET.val());
        MachineBridge.getInstance(topicNames.DATASET.val()).init(16);
		topics.put(topicNames.DATASET.val(), new TopicList() {
			@Override
			public void retrieveData(String readLine) {
				while( !(readLine = ByteSerialDataPort.getInstance().readLine()).startsWith("</"+topicNames.DATASET.val()+">") ) {
					if( readLine == null ||  readLine.length() == 0 ) {
						//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
						return;
					}
					int reading = getReadingNumber(readLine);
					double data =  getReadingValueDouble(readLine);
					//if( Props.DEBUG ) System.out.println(readLine);
					MachineReading mr = new MachineReading(1, reading, reading+1, data);
					MachineBridge.getInstance(topicNames.DATASET.val()).add(mr);
				}
			}
			@Override
			public MachineBridge getMachineBridge() {
				return MachineBridge.getInstance(topicNames.DATASET.val());
			}		
		});
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.BATTERY.val());
        MachineBridge.getInstance(topicNames.BATTERY.val()).init(16);
		topics.put(topicNames.BATTERY.val(),new TopicList() {
			@Override
			public void retrieveData(String readLine) {
				while( !(readLine = ByteSerialDataPort.getInstance().readLine()).startsWith("</"+topicNames.BATTERY.val()+">") ) {
					if( readLine == null || readLine.length() == 0 ) {
							//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
							return;
					}
					int reading = getReadingNumber(readLine);
					int data =  getReadingValueInt(readLine);
					//if( Props.DEBUG ) System.out.println(readLine);
					MachineReading mr = new MachineReading(1, reading, reading+1, data);
					MachineBridge.getInstance(topicNames.BATTERY.val()).add(mr);
				}
			}
			@Override
			public MachineBridge getMachineBridge() {
				return MachineBridge.getInstance(topicNames.BATTERY.val());
			}
		});
		// start the listener thread for this topic
		BatteryListener.getInstance();
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init "+topicNames.BATTERY.val()+" listener engaged");
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.MOTORFAULT.val());
		MachineBridge.getInstance(topicNames.MOTORFAULT.val()).init(16);
		topics.put(topicNames.MOTORFAULT.val(), new TopicList() {
			@Override
			public void retrieveData(String readLine) {		
				//for(int i = 0; i < 8; i++) {
				while( !(readLine = ByteSerialDataPort.getInstance().readLine()).startsWith("</"+topicNames.MOTORFAULT.val()+">") ) {
					if( readLine == null || readLine.length() == 0 ) {
						//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
						//continue;
						return;
					}
					int reading = getReadingNumber(readLine);
					String data =  getReadingValueString(readLine);
					//if( Props.DEBUG ) System.out.println(readLine);
					MachineReading mr = new MachineReading(1, reading, reading+1, data);
					MachineBridge.getInstance(topicNames.MOTORFAULT.val()).add(mr);
				}
			}
			@Override
			public MachineBridge getMachineBridge() {
				return MachineBridge.getInstance(topicNames.MOTORFAULT.val());
			}
		});
		MotorFaultListener.getInstance();
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init "+topicNames.MOTORFAULT.val()+" engaged");
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.ULTRASONIC.val());
		MachineBridge.getInstance(topicNames.ULTRASONIC.val()).init(16);
		topics.put(topicNames.ULTRASONIC.val(), new TopicList() {
			@Override
			public void retrieveData(String readLine) {	
				int pin = 0;
				while( !(readLine = ByteSerialDataPort.getInstance().readLine()).startsWith("</"+topicNames.ULTRASONIC.val()+">") ) {
				// get element 1 <pin>
				if( readLine == null || readLine.length() == 0 ) {
					if(DEBUG) System.out.println("Empty line returned from readLine of "+topicNames.ULTRASONIC.val());
					continue;
				}
				int reading = getReadingNumber(readLine);
				int data =  getReadingValueInt(readLine);
				pin = data;
				// get element 2 <range>
				readLine = ByteSerialDataPort.getInstance().readLine();
				if( readLine == null || readLine.length() == 0 ) {
						if(DEBUG) System.out.println("Empty line returned from readLine of "+topicNames.ULTRASONIC.val());
						continue;
				}
				reading = getReadingNumber(readLine);
				data =  (int) getReadingValueDouble(readLine);
				//if( DEBUG ) 
				//		System.out.println("Ultrasonic retrieveData pin:"+pin+"| converted:"+reading+" "+data);
				MachineReading mr = new MachineReading(1, pin, reading, data);
				MachineBridge.getInstance(topicNames.ULTRASONIC.val()).add(mr);
				}
			}
			@Override
			public MachineBridge getMachineBridge() {
				return MachineBridge.getInstance(topicNames.ULTRASONIC.val());
			}
		});
		// start an ultrasonic listener
		UltrasonicListener.getInstance();
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init "+topicNames.ULTRASONIC.val()+" engaged");
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.ANALOGPIN.val());
		MachineBridge.getInstance(topicNames.ANALOGPIN.val()).init(16);
		topics.put(topicNames.ANALOGPIN.val(), new TopicList() {
			@Override
			public void retrieveData(String readLine) {
				int pin = 0;
				while( !(readLine = ByteSerialDataPort.getInstance().readLine()).startsWith("</"+topicNames.ANALOGPIN.val()+">") ) {
				if( readLine == null || readLine.length() == 0 ) {
					System.out.println("Premature return retrieveData "+topicNames.ANALOGPIN.val());
					return;
				}
				int reading = getReadingNumber(readLine);
				int data =  getReadingValueInt(readLine);
				if( DEBUG ) 
						System.out.println(topicNames.ANALOGPIN.val()+" retrieveData:"+readLine+"| converted:"+reading+" "+data);
				pin = data;
				readLine = ByteSerialDataPort.getInstance().readLine();
				if( readLine == null || readLine.length() == 0 ) {
					System.out.println("Premature return retrieveData from empty line "+topicNames.ANALOGPIN.val());
					return;
				}
				reading = getReadingNumber(readLine);
				data =  getReadingValueInt(readLine);
				if( DEBUG ) 
					System.out.println("analog pin retrieveData:"+readLine+"| converted:"+reading+" "+data);
				MachineReading mr = new MachineReading(1, pin, reading, data);
				MachineBridge.getInstance(topicNames.ANALOGPIN.val()).add(mr);
				}
			}
			@Override
			public MachineBridge getMachineBridge() {
				return MachineBridge.getInstance(topicNames.ANALOGPIN.val());
			}
		});
		// start an analog pin listener
		AnalogPinListener.getInstance();
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init "+topicNames.ANALOGPIN.val()+" engaged");
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.DIGITALPIN.val());
		MachineBridge.getInstance(topicNames.DIGITALPIN.val()).init(32);
		topics.put(topicNames.DIGITALPIN.val(), new TopicList() {
			@Override
			public void retrieveData(String readLine) {
				int pin = 0;
				while( !(readLine = ByteSerialDataPort.getInstance().readLine()).startsWith("</"+topicNames.DIGITALPIN.val()+">") ) {
				if( readLine == null || readLine.length() == 0 ) {
					System.out.println("Premature return retrieveData pin # from empty line "+topicNames.DIGITALPIN.val());
					return;
				}
				int reading = getReadingNumber(readLine);
				int data =  getReadingValueInt(readLine);
				if( DEBUG ) 
						System.out.println(topicNames.DIGITALPIN.val()+" retrieveData:"+readLine+"| converted:"+reading+" "+data);
				pin = data;
				readLine = ByteSerialDataPort.getInstance().readLine();
				if( readLine == null || readLine.length() == 0 ) {
					System.out.println("Premature return retrieveData value from empty line "+topicNames.DIGITALPIN.val());
					return;
				}
				reading = getReadingNumber(readLine);
				data =  getReadingValueInt(readLine);
				if( DEBUG ) 
						System.out.println(topicNames.DIGITALPIN.val()+" retrieveData:"+readLine+"| converted:"+reading+" "+data);	
				MachineReading mr = new MachineReading(1, pin, reading, data);
				MachineBridge.getInstance(topicNames.DIGITALPIN.val()).add(mr);	
				}
			}
			@Override
			public MachineBridge getMachineBridge() {
				return MachineBridge.getInstance(topicNames.DIGITALPIN.val());
			}
		});
		// start a digital pin listener
		DigitalPinListener.getInstance();
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init engaged "+topicNames.DIGITALPIN.val());
		//
		// reporting functions
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.ASSIGNEDPINS.val());
		MachineBridge.getInstance(topicNames.ASSIGNEDPINS.val()).init(16);
		topics.put(topicNames.ASSIGNEDPINS.val(), new TopicList() {
			@Override
			public void retrieveData(String readLine) {  
				while( !(readLine = ByteSerialDataPort.getInstance().readLine()).startsWith("</"+topicNames.ASSIGNEDPINS.val()+">") ) {
					if( readLine == null || readLine.length() == 0 ) {
						//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
						return;
					}
					//if( Props.DEBUG ) System.out.println(readLine);
					MachineReading mr = new MachineReading(readLine);
					MachineBridge.getInstance(topicNames.ASSIGNEDPINS.val()).add(mr);
				}
			}
			@Override
			public MachineBridge getMachineBridge() {
				return MachineBridge.getInstance(topicNames.ASSIGNEDPINS.val());
			}		
		});
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.MOTORCONTROLSETTING.val());	
		MachineBridge.getInstance(topicNames.MOTORCONTROLSETTING.val()).init(128);
		topics.put(topicNames.MOTORCONTROLSETTING.val(), new TopicList() {
			@Override
			public void retrieveData(String readLine) {  
				while( !(readLine = ByteSerialDataPort.getInstance().readLine()).startsWith("</"+topicNames.MOTORCONTROLSETTING.val()+">") ) {
					if( readLine == null || readLine.length() == 0 ) {
						//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
						return;
					}
					//if( Props.DEBUG ) System.out.println(readLine);
					MachineReading mr = new MachineReading(readLine);
					MachineBridge.getInstance(topicNames.MOTORCONTROLSETTING.val()).add(mr);
				}
			}
			@Override
			public MachineBridge getMachineBridge() {
				return MachineBridge.getInstance(topicNames.MOTORCONTROLSETTING.val());
			}		
		});
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.PWMCONTROLSETTING.val());	
		MachineBridge.getInstance(topicNames.PWMCONTROLSETTING.val()).init(128);
		topics.put(topicNames.PWMCONTROLSETTING.val(), new TopicList() {
			@Override
			public void retrieveData(String readLine) {  
				while( !(readLine = ByteSerialDataPort.getInstance().readLine()).startsWith("</"+topicNames.PWMCONTROLSETTING.val()+">") ) {
					if( readLine == null || readLine.length() == 0 ) {
						//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
						return;
					}
					//if( Props.DEBUG ) System.out.println(readLine);
					MachineReading mr = new MachineReading(readLine);
					MachineBridge.getInstance(topicNames.PWMCONTROLSETTING.val()).add(mr);
				}
			}
			@Override
			public MachineBridge getMachineBridge() {
				return MachineBridge.getInstance(topicNames.PWMCONTROLSETTING.val());
			}		
		});
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.CONTROLLERSTATUS.val());			
		MachineBridge.getInstance(topicNames.CONTROLLERSTATUS.val()).init(128);
		topics.put(topicNames.CONTROLLERSTATUS.val(), new TopicList() {
			@Override
			public void retrieveData(String readLine) {  
				while( !(readLine = ByteSerialDataPort.getInstance().readLine()).startsWith("</"+topicNames.CONTROLLERSTATUS+">") ) {
						if( readLine == null || readLine.length() == 0 ) {
							//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
							return;
						}
						//if( Props.DEBUG ) System.out.println(readLine);
						MachineReading mr = new MachineReading(readLine);
						MachineBridge.getInstance(topicNames.CONTROLLERSTATUS.val()).add(mr);
				}
			}
			@Override
			public MachineBridge getMachineBridge() {
				return MachineBridge.getInstance(topicNames.CONTROLLERSTATUS.val());
			}		
		});
		
		ThreadPoolManager.getInstance().spin(getInstance(), "SYSTEM");

		if(DEBUG)
			System.out.println("AsynchDemuxer.Init END OF INITIALIZATION of Marlinspike topic listeners");
	}
	
	//
	// Methods to extract data from the line acquired from the serial port read
	//
    public synchronized double getReadingValueDouble(String readLine) {
    	if( readLine != null ) {
    		int sindex = readLine.indexOf(" ");
    		if( sindex != -1 && sindex+1 < readLine.length() ) {
    			String rnum = readLine.substring(sindex+1);
    			try {
    				return new Double(rnum).doubleValue();
    			} catch(Exception e) {
    				System.out.println("Cannot convert double value from:"+rnum);
    			}
    		}
    	}
    	System.out.println("Can't get valid Double value from:"+readLine);
    	return 0;
	}
	public synchronized int getReadingValueInt(String readLine) {
      	if( readLine != null ) {
      		int sindex = readLine.indexOf(" ");
      		if( sindex != -1 && sindex+1 < readLine.length() ) {
      			String rnum = readLine.substring(sindex+1);
      			try {
      				return new Integer(rnum).intValue();
      			} catch(Exception e) {
      				System.out.println("Cannot convert integer value from:"+rnum);
      			}
      		}
      	}
      	System.out.println("Can't get valid Integer value from:"+readLine);
      	return 0;
	}
      
    public synchronized String getReadingValueString(String readLine) {
      	if( readLine != null ) {
      		int sindex = readLine.indexOf(" ");
      		if( sindex != -1 && sindex+1 < readLine.length() ) {
      			return readLine.substring(sindex+1);
      		}
      	}
      	System.out.println("Can't get valid String from raw line:"+readLine);
      	return null;
	}
    /**
     * Get the value of the monotonically increasing reading number field
     * that precedes the pin or value reading in the given line 
     * @param readLine
     * @return The integer value of the field
     */
    public synchronized int getReadingNumber(String readLine) {
	       	if( readLine != null ) {
	       		int sindex = readLine.indexOf(" ");
      			if( sindex != -1 && sindex+1 < readLine.length() ) {
      				String rnum = readLine.substring(0,sindex);
      				try {
      					return new Integer(rnum).intValue();
      				} catch(Exception e) {
      					System.out.println("Cannot convert Integer from:"+rnum);
      				}
      			}
	       	}	
	       	System.out.println("Can't get valid reading number from:"+readLine);
	       	return 0;
	}

    // 
    // Report methods. The sequence is to issue the M-code to the MarlinSpike. The returned data will
    // include the proper <headers> which are 'demuxxed' and the correct MachineReadings are created from
    // the retrieved data and added to the queues in each MachineBridge instance for that topic
    // as they are retrieved from the MarlinSpike.<br/>
    // After issuing each M-code, call one of these methods to acquire the queue with the MachineReadings 
    // and call toString on them to build the proper output buffer for each topic, then do whatever with the String
    // payload.
    //
    /**
     * M706
     * @return A String payload of all assigned pins (if any), comma separated.
     * @throws IOException 
     */
    public synchronized String getAssignedPins() throws IOException {
		String statCommand1 = "M706"; // report all pins in use
		ByteSerialDataPort.getInstance().writeLine(statCommand1);
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {}		
    	return getReading(topicNames.ASSIGNEDPINS.val());
    }
    /**
     * M705
     * @return A String payload of motor controller configurations (if any), each one a multiline report.
     * @throws IOException 
     */
    public synchronized String getMotorControlSetting() throws IOException {
		String statCommand1 = "M705"; // report all pins in use
		ByteSerialDataPort.getInstance().writeLine(statCommand1);
		try {
			Thread.sleep(100);
		} catch (InterruptedException e) {}		
    	return getReading(topicNames.MOTORCONTROLSETTING.val());
    }
    /**
     * M798 Z<slot> X
     * @return A String payload of PWM controller status (if any), each one a multiline report.
     * @throws IOException 
     */
    public synchronized String getPWMControlSetting() throws IOException {
    	StringBuilder sb = new StringBuilder();
    	for(int i = 0; i < 10; i++) {
    		sb.append("\r\nPWM Controller in use in slot:"+i+"\r\n");
    		String statCommand1 = "M798 Z"+i+" X"; // report all pins in use
    		ByteSerialDataPort.getInstance().writeLine(statCommand1);
    		try {
    			Thread.sleep(100);
    		} catch (InterruptedException e) {}
    		sb.append(getReading(topicNames.PWMCONTROLSETTING.val()));
    		sb.append("---");
    	}
    	return sb.toString();
    }
    /**
     * M798 Z<slot>
     * @return A String payload of the status of each of the assigned motor controllers.
     * @throws IOException 
     */
    public synchronized String getControllerStatus() throws IOException {
       	StringBuilder sb = new StringBuilder();
    	for(int i = 0; i < 10; i++) {
    		sb.append("\r\nController in use in slot:"+i+"\r\n");
			if(DEBUG)
				System.out.println(this.getClass().getName()+".reportAllControllerSatus controller in use in slot"+i);
    		String statCommand1 = "M798 Z"+i; // report all pins in use
    		ByteSerialDataPort.getInstance().writeLine(statCommand1);
    		try {
    			Thread.sleep(100);
    		} catch (InterruptedException e) {}
    		sb.append(getReading(topicNames.CONTROLLERSTATUS.val()));
    		sb.append("---");
    	}
    	return sb.toString();
    }
	/**
	 * Configure the robot with a series of G-code directives at startup in file startup.gcode
	 * @throws IOException
	 */
	public synchronized void config() throws IOException {
		// now read the startup G-code directives to initiate
		try {
			ByteSerialDataPort bsdp = ByteSerialDataPort.getInstance();
			//String[] starts = FileIOUtilities.readAllLines("", "startup.gcode", ";");
			List<String> starts = FileIOUtilities.getConfig();
			for(String s : starts) {
				System.out.println("Startup GCode:"+s);
				bsdp.writeLine(s);
				Thread.sleep(100);
			}
		} catch (IOException e) {
			if( DEBUG) System.out.println("No startup.gcode file detected..");
		} catch (InterruptedException e) {}
	}
	
	public String getReading(String group) {
		synchronized(MachineBridge.class) { 
			MachineReading mr;
			StringBuilder sb = new StringBuilder();
			while((mr = MachineBridge.getInstance(group).waitForNewReading()) != null) {
				sb.append(mr.toString());
			}
			return sb.toString();
		}
	}
	
	/* (non-Javadoc)
	 * @see java.lang.Runnable#run()
	 */
	@Override
	public void run() {
		isRunning = true;
		while(shouldRun) {
			String fop, line;
			char op;
			try {
				line = ByteSerialDataPort.getInstance().readLine();
				if((op=line.charAt(0)) == '<' ) {
					int endDelim = line.indexOf('>');
					if( endDelim == -1 ) {
						System.out.println("Cannot demux received raw directive:"+op);
						continue;
					}
					fop = line.substring(1, endDelim);
					//if(DEBUG)
					//	System.out.println("op:"+op);
					TopicList tl = topics.get(fop);
					if( tl != null )
						tl.retrieveData(line);
					else
						System.out.println("Cannot retrieve topic "+fop+" from raw directive "+line);
					
				} else {
						System.out.println("Expecting directive but instead found:"+line);
						continue;
				}
	
			} catch (IndexOutOfBoundsException ioe) {
				System.out.println("AsynchDemux zero length directive, continuing..");
				continue;
			}
	
		} // shouldRun
		isRunning = false;
	}
	
	private static interface TopicList {
		public void retrieveData(String line);
		public MachineBridge getMachineBridge();
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
