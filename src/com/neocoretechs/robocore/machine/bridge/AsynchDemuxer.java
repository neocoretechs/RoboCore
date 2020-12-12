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
		STATUS("status"),
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
	private Map<String, TopicList> topics = new ConcurrentHashMap<String, TopicList>(topicNames.values().length);
	
	public TopicListInterface getTopic(String group) { return topics.get(group); }
	
	public MachineBridge getMachineBridge(String group) {
		return topics.get(group).getMachineBridge();
	}

	public synchronized void connect() throws IOException {
		ByteSerialDataPort.getInstance().connect(true);
	}
	
	public synchronized void init() {
		topicNames[] xtopics = topicNames.values();
		String[] stopics = new String[xtopics.length];
		for(int i = 0; i < xtopics.length; i++) stopics[i] = xtopics[i].val();
		ThreadPoolManager.init(stopics);
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.STATUS.val());
		topics.put(topicNames.STATUS.val(), new TopicList(topicNames.STATUS.val(),16) {
			@Override
			public void retrieveData(String readLine) {
				while( !(readLine = ByteSerialDataPort.getInstance().readLine()).startsWith("</"+topicNames.STATUS.val()+">") ) {
					if( readLine == null ||  readLine.length() == 0 ) {
						//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
						return;
					}
					//if( Props.DEBUG ) System.out.println(readLine);
					MachineReading mr = new MachineReading(readLine);
					mb.add(mr);
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		
		});
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.DATASET.val());
		topics.put(topicNames.DATASET.val(), new TopicList(topicNames.DATASET.val(), 16) {
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
					mb.add(mr);
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.toString();
			}	
		});
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.BATTERY.val());
		topics.put(topicNames.BATTERY.val(),new TopicList(topicNames.BATTERY.val(),16) {
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
					mb.add(mr);
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return new Float(((float)mr.getReadingValInt())/10.0);
			}
		});
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init "+topicNames.BATTERY.val()+" listener engaged");
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.MOTORFAULT.val());
		topics.put(topicNames.MOTORFAULT.val(), new TopicList(topicNames.MOTORFAULT.val(),16) {
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
					mb.add(mr);
				}
			}

			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init "+topicNames.MOTORFAULT.val()+" engaged");
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.ULTRASONIC.val());
		topics.put(topicNames.ULTRASONIC.val(), new TopicList(topicNames.ULTRASONIC.val(),16) {
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
				mb.add(mr);
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return new Integer(mr.getReadingValInt());
			}
			
		});
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init "+topicNames.ULTRASONIC.val()+" engaged");
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.ANALOGPIN.val());
		topics.put(topicNames.ANALOGPIN.val(), new TopicList(topicNames.ANALOGPIN.val(),16) {
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
				mb.add(mr);
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return new int[]{ mr.getRawSeq(), mr.getReadingValInt() };
			}
		});
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init "+topicNames.ANALOGPIN.val()+" engaged");
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.DIGITALPIN.val());
		topics.put(topicNames.DIGITALPIN.val(), new TopicList(topicNames.DIGITALPIN.val(),32) {
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
				mb.add(mr);	
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return new int[]{ mr.getRawSeq(), mr.getReadingValInt() };
			}
		});

		if(DEBUG)
			System.out.println("AsynchDemuxer.Init engaged "+topicNames.DIGITALPIN.val());
		//
		// reporting functions
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.ASSIGNEDPINS.val());
		topics.put(topicNames.ASSIGNEDPINS.val(), new TopicList(topicNames.ASSIGNEDPINS.val(),16) {
			@Override
			public void retrieveData(String readLine) {  
				while( !(readLine = ByteSerialDataPort.getInstance().readLine()).startsWith("</"+topicNames.ASSIGNEDPINS.val()+">") ) {
					if( readLine == null || readLine.length() == 0 ) {
						//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
						return;
					}
					//if( Props.DEBUG ) System.out.println(readLine);
					MachineReading mr = new MachineReading(readLine);
					mb.add(mr);
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.MOTORCONTROLSETTING.val());	
		topics.put(topicNames.MOTORCONTROLSETTING.val(), new TopicList(topicNames.MOTORCONTROLSETTING.val(), 128) {
			@Override
			public void retrieveData(String readLine) {  
				while( !(readLine = ByteSerialDataPort.getInstance().readLine()).startsWith("</"+topicNames.MOTORCONTROLSETTING.val()+">") ) {
					if( readLine == null || readLine.length() == 0 ) {
						//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
						return;
					}
					//if( Props.DEBUG ) System.out.println(readLine);
					MachineReading mr = new MachineReading(readLine);
					mb.add(mr);
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}	
		});
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.PWMCONTROLSETTING.val());	
		topics.put(topicNames.PWMCONTROLSETTING.val(), new TopicList(topicNames.PWMCONTROLSETTING.val(),128) {
			@Override
			public void retrieveData(String readLine) {  
				while( !(readLine = ByteSerialDataPort.getInstance().readLine()).startsWith("</"+topicNames.PWMCONTROLSETTING.val()+">") ) {
					if( readLine == null || readLine.length() == 0 ) {
						//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
						return;
					}
					//if( Props.DEBUG ) System.out.println(readLine);
					MachineReading mr = new MachineReading(readLine);
					mb.add(mr);
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}		
		});
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.CONTROLLERSTATUS.val());			
		topics.put(topicNames.CONTROLLERSTATUS.val(), new TopicList(topicNames.CONTROLLERSTATUS.val(),128) {
			@Override
			public void retrieveData(String readLine) {  
				while( !(readLine = ByteSerialDataPort.getInstance().readLine()).startsWith("</"+topicNames.CONTROLLERSTATUS+">") ) {
						if( readLine == null || readLine.length() == 0 ) {
							//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
							return;
						}
						//if( Props.DEBUG ) System.out.println(readLine);
						MachineReading mr = new MachineReading(readLine);
						mb.add(mr);
				}
			}

			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
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
	
	
	/**
	 * Asynchronous demuxxing main loop.
	 * Using {@see ByteSerialDataPort}, read a line from the Marlinspike and check the incoming header
	 * format. If it complies with the expected header, dispatch it to the proper {@code TopicListInterface#retrieveData(String)}
	 * by first looking it up in the {@link TopicList}.
	 * {@see java.lang.Runnable#run()}
	 */
	@Override
	public void run() {
		isRunning = true;
		while(shouldRun) {
			String fop, line;
			char op;
			try {
				line = ByteSerialDataPort.getInstance().readLine();
				if(DEBUG)
					System.out.println(this.getClass().getName()+" main read loop readLine:"+line);
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
	
	public static interface TopicListInterface {
		public void retrieveData(String line);
		public MachineBridge getMachineBridge();
		public Object getResult(MachineReading mr);
	}
	private static abstract class TopicList implements TopicListInterface {
		MachineBridge mb;
		TopicList(String groupName, int queueSize) {
			mb = new MachineBridge(groupName, queueSize);
		}
		public MachineBridge getMachineBridge() { return mb; }
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
