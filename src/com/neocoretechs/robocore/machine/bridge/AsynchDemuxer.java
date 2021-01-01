package com.neocoretechs.robocore.machine.bridge;

import java.io.IOException;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import com.neocoretechs.robocore.ThreadPoolManager;
import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;
import com.neocoretechs.robocore.serialreader.DataPortInterface;

/**
 * This class is the primary interface between real time data and the other subsystems.
 * Its primary function is to demultiplex the input stream coming from {@code DataPortInterface} 
 * data sources such as the attached microcontroller Mega2560 etc that implement the interface and 
 * utilize a protocol with '<header>',line number, data value.
 * Each 'topic' described by the demultiplexed header as it flows in with its associated data is expected to have:
 * OPTIONAL:
 * 1) A thread that services the final processing listener class and deque for the given topic '<header>'
 * 2) A listener class that is serviced by the above thread that takes raw MachineReadings and transforms them if necessary
 * MANDATORY:
 * 3) An instance of {@code DataPortInterface} to connect to.
 * 4) A 'TopicList' class in the hash table with its associated 'header'
 * 5) An instance of 'MachineBridge' that operates on the raw data for a given topic 'header'
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
	private DataPortInterface dataPort;
	static Object mutexWrite = new Object();
	private final static String MSG_BEGIN = "<";
	private final static String MSG_TERMINATE ="/>";

	public enum topicNames {
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
		TIME("time"),//time report: <time>,newline,"1 Imin Isec"</time>"
		CONTROLLERSTATUS("controllerstatus"),
		CONTROLLERSTOPPED("Controller stopped due to errors"),
		NOMORGCODE("Neither G nor M code found "),
		BADMOTOR("Bad Motor command "), // will be followed by " status motorchannel motorpower/>"
		BADPWM("Bad PWM Driver command "), // will be followed by "status motorchannel pwmlevel/>"
		UNKNOWNG("Unknown G code "),
		UNKNOWNM("Unknown M code "),
		BADCONTROL("BAD CONTROLLER TYPE:"), // comes from M10, followed by "badtype/>"
		NOCHECKSUM("No Checksum with line number, Last Line: "),
		NOLINECHECK("No Line Number with checksum, Last Line: "),
		CHECKMISMATCH("checksum mismatch, Last Line: "),
		LINESEQ("Line Number is not Last Line Number+1, Last Line: "),
		G4("G4"),G5("G5"),G99("G99"),G100("G100"),
		M0("M0"),M1("M1"),M2("M2"),M3("M3"),M4("M4"),M5("M5"),M6("M6"),M7("M7"),M8("M8"),M9("M9"),M10("M10"),M11("M11"),M12("M12"),
		M33("M33"),M35("M35"),M36("M36"),M37("M37"),M38("M38"),M39("M39"),M40("M40"),M41("M41"),M42("M42"),M44("M44"),M45("M45"),M46("M46"),
		M80("M80"),M81("M81"),M301("M301"),M302("M302"),M304("M304"),M306("M306"),M444("M444"),
		M445("M445"),M500("M500"),M501("M501"),M502("M502"),M503("M503"),M799("M799"),M999("M999"),
		M115("FIRMWARE_NAME:Marlinspike RoboCore"); // followed by FIRMWARE_URL,PROTOCOL_VERSION,MACHINE_TYPE,MACHINE NAME,MACHINE_UUID
		String name;
		topicNames(String name) { this.name = name;}
		public String val() { return name; }
	};
	
	private Map<String, TopicList> topics = new ConcurrentHashMap<String, TopicList>(topicNames.values().length);
	public TopicListInterface getTopic(String group) { return topics.get(group); }
	
	private CircularBlockingDeque<String> marlinLines = new CircularBlockingDeque<String>(256);
	public void clearLineBuffer() { marlinLines.clear(); }
	private CircularBlockingDeque<String> toWrite = new CircularBlockingDeque<String>(256);
	public void clearWriteBuffer() { toWrite.clear(); }
	public static void addWrite(AsynchDemuxer ad, String req) { 
		boolean overwrite = ad.toWrite.addLast(req);
		if(overwrite)
			System.out.println("WARNING - OUBOUND MARLINSPIKE QUEUE OVERWRITE!");
	}
	public String takeWrite() {
		try {
			return toWrite.takeFirst();
		} catch (InterruptedException e) {
			e.printStackTrace();
			return null;
		}
	}
	public MachineBridge getMachineBridge(String group) {
		return topics.get(group).getMachineBridge();
	}

	public synchronized void connect(DataPortInterface dataPort) throws IOException {
		this.dataPort = dataPort;
		dataPort.connect(true);
	}
	
	public DataPortInterface getDataPort() { return dataPort; }
	
	public synchronized void init() {
		topicNames[] xtopics = topicNames.values();
		String[] stopics = new String[xtopics.length];
		for(int i = 0; i < xtopics.length; i++) stopics[i] = xtopics[i].val();
		ThreadPoolManager.init(stopics);
		//
		// G4
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.G4.val());
		topics.put(topicNames.G4.val(), new TopicList(this, topicNames.G4.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// G5
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.G5.val());
		topics.put(topicNames.G5.val(), new TopicList(this, topicNames.G5.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// G99
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.G99.val());
		topics.put(topicNames.G99.val(), new TopicList(this, topicNames.G99.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// G100
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.G100.val());
		topics.put(topicNames.G100.val(), new TopicList(this, topicNames.G100.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M0
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M0.val());
		topics.put(topicNames.M0.val(), new TopicList(this, topicNames.M0.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M1
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M1.val());
		topics.put(topicNames.M1.val(), new TopicList(this, topicNames.M1.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M2
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M2.val());
		topics.put(topicNames.M2.val(), new TopicList(this, topicNames.M2.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M3
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M3.val());
		topics.put(topicNames.M3.val(), new TopicList(this, topicNames.M3.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M4
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M4.val());
		topics.put(topicNames.M4.val(), new TopicList(this, topicNames.M4.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M5
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M5.val());
		topics.put(topicNames.M5.val(), new TopicList(this, topicNames.M5.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M6
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M6.val());
		topics.put(topicNames.M6.val(), new TopicList(this, topicNames.M6.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M7
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M7.val());
		topics.put(topicNames.M7.val(), new TopicList(this, topicNames.M7.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M8
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M8.val());
		topics.put(topicNames.M8.val(), new TopicList(this, topicNames.M8.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M9
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M9.val());
		topics.put(topicNames.M9.val(), new TopicList(this, topicNames.M9.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M10
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M10.val());
		topics.put(topicNames.M10.val(), new TopicList(this, topicNames.M10.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M101
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M11.val());
		topics.put(topicNames.M11.val(), new TopicList(this, topicNames.M11.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M12
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M12.val());
		topics.put(topicNames.M12.val(), new TopicList(this, topicNames.M12.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M33
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M33.val());
		topics.put(topicNames.M33.val(), new TopicList(this, topicNames.M33.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M35
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M35.val());
		topics.put(topicNames.M35.val(), new TopicList(this, topicNames.M35.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M36
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M36.val());
		topics.put(topicNames.M36.val(), new TopicList(this, topicNames.M36.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M37
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M37.val());
		topics.put(topicNames.M37.val(), new TopicList(this, topicNames.M37.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M38
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M38.val());
		topics.put(topicNames.M38.val(), new TopicList(this, topicNames.M38.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M39
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M39.val());
		topics.put(topicNames.M39.val(), new TopicList(this, topicNames.M39.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M10
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M40.val());
		topics.put(topicNames.M40.val(), new TopicList(this, topicNames.M40.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M41
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M41.val());
		topics.put(topicNames.M41.val(), new TopicList(this, topicNames.M41.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M42
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M42.val());
		topics.put(topicNames.M42.val(), new TopicList(this, topicNames.M42.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M44
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M44.val());
		topics.put(topicNames.M44.val(), new TopicList(this, topicNames.M44.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M45
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M45.val());
		topics.put(topicNames.M45.val(), new TopicList(this, topicNames.M45.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M46
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M46.val());
		topics.put(topicNames.M46.val(), new TopicList(this, topicNames.M46.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M80
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M80.val());
		topics.put(topicNames.M80.val(), new TopicList(this, topicNames.M80.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M81
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M81.val());
		topics.put(topicNames.M81.val(), new TopicList(this, topicNames.M81.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M301
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M301.val());
		topics.put(topicNames.M301.val(), new TopicList(this, topicNames.M301.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M302
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M302.val());
		topics.put(topicNames.M302.val(), new TopicList(this, topicNames.M302.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M304
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M304.val());
		topics.put(topicNames.M304.val(), new TopicList(this, topicNames.M304.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M306
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M306.val());
		topics.put(topicNames.M306.val(), new TopicList(this, topicNames.M306.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M444
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M444.val());
		topics.put(topicNames.M444.val(), new TopicList(this, topicNames.M444.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M445
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M445.val());
		topics.put(topicNames.M445.val(), new TopicList(this, topicNames.M445.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M500
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M500.val());
		topics.put(topicNames.M500.val(), new TopicList(this, topicNames.M500.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M501
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M501.val());
		topics.put(topicNames.M501.val(), new TopicList(this, topicNames.M501.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M502
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M502.val());
		topics.put(topicNames.M502.val(), new TopicList(this, topicNames.M502.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M503
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M503.val());
		topics.put(topicNames.M503.val(), new TopicList(this, topicNames.M503.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M799
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M799.val());
		topics.put(topicNames.M799.val(), new TopicList(this, topicNames.M799.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// M999
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M999.val());
		topics.put(topicNames.M999.val(), new TopicList(this, topicNames.M999.val(),2) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// status - M700
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.STATUS.val());
		topics.put(topicNames.STATUS.val(), new TopicList(this, topicNames.STATUS.val(),16) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				while( !isLineTerminal(readLine) ) {
					readLine = marlinLines.takeFirst();
					if( readLine == null ||  readLine.length() == 0 ) {
						if(DEBUG)System.out.println(this.getClass().getName()+".retrieveData: premature EOR");
						break;
					}
					if( DEBUG ) 
						System.out.println(this.getClass().getName()+".retrieveData:"+readLine);
					MachineReading mr = new MachineReading(readLine);
					mb.add(mr);
				}
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// Dataset
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.DATASET.val());
		topics.put(topicNames.DATASET.val(), new TopicList(this, topicNames.DATASET.val(), 16) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				while( !isLineTerminal(readLine) ) {
					readLine = marlinLines.takeFirst();
					if( readLine == null ||  readLine.length() == 0 ) {
						//if(DEBUG)System.out.println("Empty line returned from readLine");
						break;
					}
					readLine = extractPayload(readLine, topicNames.DATASET.val());
					int reading = getReadingNumber(readLine);
					double data =  getReadingValueDouble(readLine);
					//if( DEBUG ) System.out.println(readLine);
					MachineReading mr = new MachineReading(1, reading, reading+1, data);
					mb.add(mr);		
				}
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.toString();
			}	
		});
		//
		// Battery
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.BATTERY.val());
		topics.put(topicNames.BATTERY.val(),new TopicList(this, topicNames.BATTERY.val(),16) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				String sMarker = MSG_BEGIN+topicNames.BATTERY.val()+MSG_TERMINATE;
				// Account for payloads on one line, delimited by our markers, or multiple lines with our markers as prefix and suffix.
				// If we are here, we know the line begins with our marker header, but is there additional data on the line?
				if(readLine.length() > sMarker.length()) {
					MachineReading mr = new MachineReading(readLine.substring(sMarker.length(),readLine.length()));
					mb.add(mr);
				}
				while( !(readLine = marlinLines.takeFirst()).startsWith(sMarker) ) {
					if( readLine == null || readLine.length() == 0 ) {
							//if(DEBUG)System.out.println("Empty line returned from readLine");
							break;
					}
					// Is our delimiting marker part of a one-line payload, or used at the end of a multiline payload?
					if(readLine.endsWith(sMarker)) {
						int reading = getReadingNumber( readLine.substring(0,readLine.length()-sMarker.length()));
						int data =  getReadingValueInt( readLine.substring(0,readLine.length()-sMarker.length()));
						MachineReading mr = new MachineReading(1, reading, reading+1, data);
						mb.add(mr);
						break;
					} else {
						int reading = getReadingNumber(readLine);
						int data =  getReadingValueInt(readLine);
						//if( DEBUG ) System.out.println(readLine);
						MachineReading mr = new MachineReading(1, reading, reading+1, data);
						mb.add(mr);
					}
				}
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return new Float(((float)mr.getReadingValInt())/10.0);
			}
		});
		//
		// Motorfault
		//                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
		if(DEBUG) {
			System.out.println("AsynchDemuxer.Init "+topicNames.BATTERY.val()+" engaged");
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.MOTORFAULT.val());
		}
		topics.put(topicNames.MOTORFAULT.val(), new TopicList(this, topicNames.MOTORFAULT.val(),16) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {		
				while( !isLineTerminal(readLine) ) {
					readLine = marlinLines.takeFirst();
					if( readLine == null || readLine.length() == 0 ) {
						//if(DEBUG)System.out.println("Empty line returned from readLine");
						//continue;
						break;
					}
					// Is our delimiting marker part of a one-line payload, or used at the end of a multiline payload?
					int reading = getReadingNumber( extractPayload(readLine, topicNames.MOTORFAULT.val()));
					String data =  getReadingValueString( extractPayload(readLine, topicNames.MOTORFAULT.val()));
					MachineReading mr = new MachineReading(1, reading, reading+1, data);
					mb.add(mr);
				}
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// Ultrasonic
		//
		if(DEBUG) {
			System.out.println("AsynchDemuxer.Init "+topicNames.MOTORFAULT.val()+" engaged");
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.ULTRASONIC.val());
		}
		topics.put(topicNames.ULTRASONIC.val(), new TopicList(this, topicNames.ULTRASONIC.val(),16) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {	
				int pin = 0, reading = 0, data = 0;
				String sMarker = MSG_BEGIN+topicNames.ULTRASONIC.val()+MSG_TERMINATE;
				// Account for payloads on one line, delimited by our markers, or multiple lines with our markers as prefix and suffix.
				// If we are here, we know the line begins with our marker header, but is there additional data on the line?
				if(readLine.length() > sMarker.length()) {
					pin =  getReadingValueInt(readLine.substring(sMarker.length(),readLine.length()));                             
				}
				while( !(readLine = marlinLines.takeFirst()).startsWith(sMarker) ) {
					if( readLine == null || readLine.length() == 0 ) {
							if(DEBUG) System.out.println("Empty line returned from readLine of "+topicNames.ULTRASONIC.val());
							break;
					}
					if(readLine.endsWith(sMarker)) {
						reading = getReadingNumber(readLine.substring(sMarker.length(),readLine.length()));
						data =  getReadingValueInt(readLine.substring(sMarker.length(),readLine.length()));
						if(reading == 1) {
							System.out.println("Malformed request for "+topicNames.ULTRASONIC.val()+":"+readLine);
							continue;
						}
						MachineReading mr = new MachineReading(1, pin, reading, data);
						mb.add(mr);
						break;
					} else {
						//if( DEBUG ) 
						//		System.out.println("Ultrasonic retrieveData pin:"+pin+"| converted:"+reading+" "+data);
						reading = getReadingNumber(readLine);
						data =  getReadingValueInt(readLine);
						if(reading == 1) {
							pin = data;
						} else {
							MachineReading mr = new MachineReading(1, pin, reading, data);
							mb.add(mr);
						}
					}
				}
				if( DEBUG ) 
					System.out.println(topicNames.ULTRASONIC.val()+" retrieveData:"+readLine+"| converted:"+reading+" "+data);
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return new Integer(mr.getReadingValInt());
			}
			
		});
		//
		// Analogpin
		//
		if(DEBUG) {
			System.out.println("AsynchDemuxer.Init "+topicNames.ULTRASONIC.val()+" engaged");
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.ANALOGPIN.val());
		}
		topics.put(topicNames.ANALOGPIN.val(), new TopicList(this, topicNames.ANALOGPIN.val(),16) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				int pin = 0;
				String sMarker = MSG_BEGIN+topicNames.ANALOGPIN.val()+MSG_TERMINATE;
				// Account for payloads on one line, delimited by our markers, or multiple lines with our markers as prefix and suffix.
				// If we are here, we know the line begins with our marker header, but is there additional data on the line?
				if(readLine.length() > sMarker.length()) {
					pin = getReadingValueInt(readLine.substring(sMarker.length(),readLine.length()));
				}
				while( !(readLine = marlinLines.takeFirst()).startsWith(sMarker) ) {
					if( readLine == null || readLine.length() == 0 ) {
						System.out.println("Premature return retrieveData "+topicNames.ANALOGPIN.val());
						break;
					}
					int reading = 0, data = 0;
					//if( DEBUG ) 
					//	System.out.println(topicNames.ANALOGPIN.val()+" retrieveData:"+readLine+"| converted:"+reading+" "+data);
					// Is our delimiting marker part of a one-line payload, or used at the end of a multiline payload?
					if(readLine.endsWith(sMarker)) {
						reading = getReadingNumber( readLine.substring(0,readLine.length()-sMarker.length()));
						data =  getReadingValueInt( readLine.substring(0,readLine.length()-sMarker.length()));
						if(reading == 1) {
							System.out.println("Malformed request for "+topicNames.ANALOGPIN.val()+":"+readLine);
							continue;
						}
						MachineReading mr = new MachineReading(1, pin, reading, data);
						mb.add(mr);
						break;
					} else {
						reading = getReadingNumber(readLine);
						data =  getReadingValueInt(readLine);
						if(reading == 1) {
							pin = data;
						} else {
							MachineReading mr = new MachineReading(1, pin, reading, data);
							mb.add(mr);
						}
						if( DEBUG ) 
							System.out.println("analog pin retrieveData:"+readLine+"| converted:"+reading+" "+data);
					}
				}
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return new int[]{ mr.getRawSeq(), mr.getReadingValInt() };
			}
		});
		//
		// Digitalpin
		//
		if(DEBUG) {
			System.out.println("AsynchDemuxer.Init "+topicNames.ANALOGPIN.val()+" engaged");
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.DIGITALPIN.val());
		}
		topics.put(topicNames.DIGITALPIN.val(), new TopicList(this, topicNames.DIGITALPIN.val(),32) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				int reading = 0, data = 0;
				int pin = 0;
				String sMarker = MSG_BEGIN+topicNames.DIGITALPIN.val()+MSG_TERMINATE;
				// Account for payloads on one line, delimited by our markers, or multiple lines with our markers as prefix and suffix.
				// If we are here, we know the line begins with our marker header, but is there additional data on the line?
				if(readLine.length() > sMarker.length()) {
					pin =  getReadingValueInt( readLine.substring(0,readLine.length()-sMarker.length()));
				}
				while( !(readLine = marlinLines.takeFirst()).startsWith(sMarker) ) {
					if( readLine == null || readLine.length() == 0 ) {
						System.out.println("Premature return retrieveData pin # from empty line "+topicNames.DIGITALPIN.val());
						break;
					}
					// Is our delimiting marker part of a one-line payload, or used at the end of a multiline payload?
					if(readLine.endsWith(sMarker)) {
						reading = getReadingNumber( readLine.substring(0,readLine.length()-sMarker.length()));
						data =  getReadingValueInt( readLine.substring(0,readLine.length()-sMarker.length()));
						if(reading == 1) {
							System.out.println("Malformed request for "+topicNames.DIGITALPIN.val()+":"+readLine);
							continue;
						}
						MachineReading mr = new MachineReading(1, pin, reading, data);
						mb.add(mr);
						break;
					} else {
						reading = getReadingNumber(readLine);
						data =  getReadingValueInt(readLine);
						if( DEBUG ) 
							System.out.println(topicNames.DIGITALPIN.val()+" retrieveData:"+readLine+"| converted:"+reading+" "+data);
						if(reading == 1) {
							pin = data;
						} else {
							MachineReading mr = new MachineReading(1, pin, reading, data);
							mb.add(mr);
						}
					}
				}
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
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
		// reporting functions...
		// Assigned pins
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.ASSIGNEDPINS.val());
		topics.put(topicNames.ASSIGNEDPINS.val(), new TopicList(this, topicNames.ASSIGNEDPINS.val(),16) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {  
				String sMarker = MSG_BEGIN+topicNames.ASSIGNEDPINS.val()+MSG_TERMINATE;
				// Account for payloads on one line, delimited by our markers, or multiple lines with our markers as prefix and suffix.
				// If we are here, we know the line begins with our marker header, but is there additional data on the line?
				if(readLine.length() > sMarker.length()) {
					MachineReading mr = new MachineReading(readLine.substring(sMarker.length(),readLine.length()));
					mb.add(mr);
				}
				while( !(readLine = marlinLines.takeFirst()).startsWith(sMarker) ) {
					if( readLine == null || readLine.length() == 0 ) {
						//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
						break;
					}
					if(DEBUG) 
						System.out.println("AssignedPins subretrieval:"+readLine);
					// Is our delimiting marker part of a one-line payload, or used at the end of a multiline payload?
					if(readLine.endsWith(sMarker)) {
						MachineReading mr = new MachineReading(readLine.substring(0,readLine.length()-sMarker.length()));
						mb.add(mr);
						break;
					}
					MachineReading mr = new MachineReading(readLine);
					mb.add(mr);
				}
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}
		});
		//
		// Motorcontrol
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.MOTORCONTROLSETTING.val());	
		topics.put(topicNames.MOTORCONTROLSETTING.val(), new TopicList(this, topicNames.MOTORCONTROLSETTING.val(), 128) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				String sMarker = MSG_BEGIN+topicNames.MOTORCONTROLSETTING.val()+MSG_TERMINATE;
				// Account for payloads on one line, delimited by our markers, or multiple lines with our markers as prefix and suffix.
				// If we are here, we know the line begins with our marker header, but is there additional data on the line?
				if(readLine.length() > sMarker.length()) {
					MachineReading mr = new MachineReading(readLine.substring(sMarker.length(),readLine.length()));
					mb.add(mr);
				}
				while( !(readLine = marlinLines.takeFirst()).startsWith(sMarker) ) {
					if( readLine == null || readLine.length() == 0 ) {
						//if(DEBUG)System.out.println("Empty line returned from readLine");
						break;
					}
					// Is our delimiting marker part of a one-line payload, or used at the end of a multiline payload?
					if(readLine.endsWith(sMarker)) {
						MachineReading mr = new MachineReading(readLine.substring(0,readLine.length()-sMarker.length()));
						mb.add(mr);
						break;
					}
					//if( DEBUG ) System.out.println(readLine);
					MachineReading mr = new MachineReading(readLine);
					mb.add(mr);
				}
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}	
		});
		//
		// PWM control
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.PWMCONTROLSETTING.val());	
		topics.put(topicNames.PWMCONTROLSETTING.val(), new TopicList(this, topicNames.PWMCONTROLSETTING.val(),128) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				String sMarker = MSG_BEGIN+topicNames.PWMCONTROLSETTING.val()+MSG_TERMINATE;
				// Account for payloads on one line, delimited by our markers, or multiple lines with our markers as prefix and suffix.
				// If we are here, we know the line begins with our marker header, but is there additional data on the line?
				if(readLine.length() > sMarker.length()) {
					MachineReading mr = new MachineReading(readLine.substring(sMarker.length(),readLine.length()));
					mb.add(mr);
				}
				while( !(readLine = marlinLines.takeFirst()).startsWith(sMarker) ) {
					if( readLine == null || readLine.length() == 0 ) {
						//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
						break;
					}
					// Is our delimiting marker part of a one-line payload, or used at the end of a multiline payload?
					if(readLine.endsWith(sMarker)) {
						MachineReading mr = new MachineReading(readLine.substring(0,readLine.length()-sMarker.length()));
						mb.add(mr);
						break;
					}
					//if( DEBUG ) System.out.println(readLine);
					MachineReading mr = new MachineReading(readLine);
					mb.add(mr);
				}
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}		
		});
		//
		// Controller status
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.CONTROLLERSTATUS.val());			
		topics.put(topicNames.CONTROLLERSTATUS.val(), new TopicList(this, topicNames.CONTROLLERSTATUS.val(),128) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				String sMarker = MSG_BEGIN+topicNames.CONTROLLERSTATUS.val()+MSG_TERMINATE;
				// Account for payloads on one line, delimited by our markers, or multiple lines with our markers as prefix and suffix.
				// If we are here, we know the line begins with our marker header, but is there additional data on the line?
				if(readLine.length() > sMarker.length()) {
					MachineReading mr = new MachineReading(readLine.substring(sMarker.length(),readLine.length()));
					mb.add(mr);
				}
				while( !(readLine = marlinLines.takeFirst()).startsWith(sMarker) ) {
						if( readLine == null || readLine.length() == 0 ) {
							//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
							break;
						}
						// Is our delimiting marker part of a one-line payload, or used at the end of a multiline payload?
						if(readLine.endsWith(sMarker)) {
							MachineReading mr = new MachineReading(readLine.substring(0,readLine.length()-sMarker.length()));
							mb.add(mr);
							break;
						}
						//if( Props.DEBUG ) System.out.println(readLine);
						MachineReading mr = new MachineReading(readLine);
						mb.add(mr);
				}
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}

			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}	
		});
		//
		// time
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.TIME.val());			
		topics.put(topicNames.TIME.val(), new TopicList(this, topicNames.TIME.val(),16) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				String sMarker = MSG_BEGIN+topicNames.TIME.val()+MSG_TERMINATE;
				// Account for payloads on one line, delimited by our markers, or multiple lines with our markers as prefix and suffix.
				// If we are here, we know the line begins with our marker header, but is there additional data on the line?
				if(readLine.length() > sMarker.length()) {
					MachineReading mr = new MachineReading(readLine.substring(sMarker.length(),readLine.length()));
					mb.add(mr);
				}
				while( !(readLine = marlinLines.takeFirst()).startsWith(sMarker) ) {
						if( readLine == null || readLine.length() == 0 ) {
							//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
							break;
						}
						// Is our delimiting marker part of a one-line payload, or used at the end of a multiline payload?
						if(readLine.endsWith(sMarker)) {
							MachineReading mr = new MachineReading(readLine.substring(0,readLine.length()-sMarker.length()));
							mb.add(mr);
							break;
						}
						//if( Props.DEBUG ) System.out.println(readLine);
						MachineReading mr = new MachineReading(readLine);
						mb.add(mr);
				}
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}

			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}	
		});
		//
		// Controller stopped
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.CONTROLLERSTOPPED.val());			
		topics.put(topicNames.CONTROLLERSTOPPED.val(), new TopicList(this, topicNames.CONTROLLERSTOPPED.val(),16) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				String sMarker = MSG_BEGIN+topicNames.CONTROLLERSTOPPED.val()+MSG_TERMINATE;
				// Account for payloads on one line, delimited by our markers, or multiple lines with our markers as prefix and suffix.
				// If we are here, we know the line begins with our marker header, but is there additional data on the line?
				if(readLine.length() > sMarker.length()) {
					MachineReading mr = new MachineReading(readLine.substring(sMarker.length(),readLine.length()));
					mb.add(mr);
				}
				while( !(readLine = marlinLines.takeFirst()).startsWith(sMarker) ) {
						if( readLine == null || readLine.length() == 0 ) {
							//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
							break;
						}
						// Is our delimiting marker part of a one-line payload, or used at the end of a multiline payload?
						if(readLine.endsWith(sMarker)) {
							MachineReading mr = new MachineReading(readLine.substring(0,readLine.length()-sMarker.length()));
							mb.add(mr);
							break;
						}
						//if( Props.DEBUG ) System.out.println(readLine);
						MachineReading mr = new MachineReading(readLine);
						mb.add(mr);
				}
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}

			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}	
		});
		//
		// No M or G code
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.NOMORGCODE.val());			
		topics.put(topicNames.NOMORGCODE.val(), new TopicList(this, topicNames.NOMORGCODE.val(),16) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				String sMarker = MSG_BEGIN+topicNames.NOMORGCODE.val()+MSG_TERMINATE;
				// Account for payloads on one line, delimited by our markers, or multiple lines with our markers as prefix and suffix.
				// If we are here, we know the line begins with our marker header, but is there additional data on the line?
				if(readLine.length() > sMarker.length()) {
					MachineReading mr = new MachineReading(readLine.substring(sMarker.length(),readLine.length()));
					mb.add(mr);
				}
				while( !(readLine = marlinLines.takeFirst()).startsWith(sMarker) ) {
						if( readLine == null || readLine.length() == 0 ) {
							//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
							break;
						}
						// Is our delimiting marker part of a one-line payload, or used at the end of a multiline payload?
						if(readLine.endsWith(sMarker)) {
							MachineReading mr = new MachineReading(readLine.substring(0,readLine.length()-sMarker.length()));
							mb.add(mr);
							break;
						}
						//if( Props.DEBUG ) System.out.println(readLine);
						MachineReading mr = new MachineReading(readLine);
						mb.add(mr);
				}
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}

			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}	
		});
		//
		// Bad motor
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.BADMOTOR.val());			
		topics.put(topicNames.BADMOTOR.val(), new TopicList(this, topicNames.BADMOTOR.val(),16) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				String sMarker = MSG_BEGIN+topicNames.BADMOTOR.val();
				// Account for payloads on one line, delimited by our markers, or multiple lines with our markers as prefix and suffix.
				// If we are here, we know the line begins with our marker header, but is there additional data on the line?
				if(readLine.length() > sMarker.length()) {
					MachineReading mr = new MachineReading(readLine.substring(sMarker.length(),readLine.length()));
					mb.add(mr);
				}
				while( !(readLine = marlinLines.takeFirst()).startsWith(sMarker) ) {
						if( readLine == null || readLine.length() == 0 ) {
							//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
							break;
						}
						// Is our delimiting marker part of a one-line payload, or used at the end of a multiline payload?
						if(readLine.endsWith(sMarker)) {
							MachineReading mr = new MachineReading(readLine.substring(0,readLine.length()-sMarker.length()));
							mb.add(mr);
							break;
						}
						//if( Props.DEBUG ) System.out.println(readLine);
						MachineReading mr = new MachineReading(readLine);
						mb.add(mr);
				}
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}

			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}	
		});
		//
		// Bad PWM
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.BADPWM.val());			
		topics.put(topicNames.BADPWM.val(), new TopicList(this, topicNames.BADPWM.val(),128) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				String sMarker = MSG_BEGIN+topicNames.BADPWM.val()+MSG_TERMINATE;
				// Account for payloads on one line, delimited by our markers, or multiple lines with our markers as prefix and suffix.
				// If we are here, we know the line begins with our marker header, but is there additional data on the line?
				if(readLine.length() > sMarker.length()) {
					MachineReading mr = new MachineReading(readLine.substring(sMarker.length(),readLine.length()));
					mb.add(mr);
				}
				while( !(readLine = marlinLines.takeFirst()).startsWith(sMarker) ) {
						if( readLine == null || readLine.length() == 0 ) {
							//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
							break;
						}
						// Is our delimiting marker part of a one-line payload, or used at the end of a multiline payload?
						if(readLine.endsWith(sMarker)) {
							MachineReading mr = new MachineReading(readLine.substring(0,readLine.length()-sMarker.length()));
							mb.add(mr);
							break;
						}
						//if( Props.DEBUG ) System.out.println(readLine);
						MachineReading mr = new MachineReading(readLine);
						mb.add(mr);
				}
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}
			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}	
		});
		//
		// Unknown G code
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.UNKNOWNG.val());			
		topics.put(topicNames.UNKNOWNG.val(), new TopicList(this, topicNames.UNKNOWNG.val(),16) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				String sMarker = MSG_BEGIN+topicNames.UNKNOWNG.val()+MSG_TERMINATE;
				// Account for payloads on one line, delimited by our markers, or multiple lines with our markers as prefix and suffix.
				// If we are here, we know the line begins with our marker header, but is there additional data on the line?
				if(readLine.length() > sMarker.length()) {
					MachineReading mr = new MachineReading(readLine.substring(sMarker.length(),readLine.length()));
					mb.add(mr);
				}
				while( !(readLine = marlinLines.takeFirst()).startsWith(sMarker) ) {
						if( readLine == null || readLine.length() == 0 ) {
							//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
							break;
						}
						// Is our delimiting marker part of a one-line payload, or used at the end of a multiline payload?
						if(readLine.endsWith(sMarker)) {
							MachineReading mr = new MachineReading(readLine.substring(0,readLine.length()-sMarker.length()));
							mb.add(mr);
							break;
						}
						//if( Props.DEBUG ) System.out.println(readLine);
						MachineReading mr = new MachineReading(readLine);
						mb.add(mr);
				}
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}

			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}	
		});
		//
		// Unknown M code
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.UNKNOWNM.val());			
		topics.put(topicNames.UNKNOWNM.val(), new TopicList(this, topicNames.UNKNOWNM.val(),16) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				String sMarker = MSG_BEGIN+topicNames.UNKNOWNM.val()+MSG_TERMINATE;
				// Account for payloads on one line, delimited by our markers, or multiple lines with our markers as prefix and suffix.
				// If we are here, we know the line begins with our marker header, but is there additional data on the line?
				if(readLine.length() > sMarker.length()) {
					MachineReading mr = new MachineReading(readLine.substring(sMarker.length(),readLine.length()));
					mb.add(mr);
				}
				while( !(readLine = marlinLines.takeFirst()).startsWith(sMarker) ) {
						if( readLine == null || readLine.length() == 0 ) {
							//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
							break;
						}
						// Is our delimiting marker part of a one-line payload, or used at the end of a multiline payload?
						if(readLine.endsWith(sMarker)) {
							MachineReading mr = new MachineReading(readLine.substring(0,readLine.length()-sMarker.length()));
							mb.add(mr);
							break;
						}
						//if( Props.DEBUG ) System.out.println(readLine);
						MachineReading mr = new MachineReading(readLine);
						mb.add(mr);
				}
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}

			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}	
		});
		//
		// Bad Control
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.BADCONTROL.val());			
		topics.put(topicNames.BADCONTROL.val(), new TopicList(this, topicNames.BADCONTROL.val(),16) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				String sMarker = MSG_BEGIN+topicNames.BADCONTROL.val()+MSG_TERMINATE;
				// Account for payloads on one line, delimited by our markers, or multiple lines with our markers as prefix and suffix.
				// If we are here, we know the line begins with our marker header, but is there additional data on the line?
				if(readLine.length() > sMarker.length()) {
					MachineReading mr = new MachineReading(readLine.substring(sMarker.length(),readLine.length()));
					mb.add(mr);
				}
				while( !(readLine = marlinLines.takeFirst()).startsWith(sMarker) ) {
						if( readLine == null || readLine.length() == 0 ) {
							//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
							break;
						}
						// Is our delimiting marker part of a one-line payload, or used at the end of a multiline payload?
						if(readLine.endsWith(sMarker)) {
							MachineReading mr = new MachineReading(readLine.substring(0,readLine.length()-sMarker.length()));
							mb.add(mr);
							break;
						}
						//if( Props.DEBUG ) System.out.println(readLine);
						MachineReading mr = new MachineReading(readLine);
						mb.add(mr);
				}
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}

			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}	
		});
		//
		// No checksum
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.NOCHECKSUM.val());			
		topics.put(topicNames.NOCHECKSUM.val(), new TopicList(this, topicNames.NOCHECKSUM.val(),16) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				String sMarker = MSG_BEGIN+topicNames.NOCHECKSUM.val()+MSG_TERMINATE;
				// Account for payloads on one line, delimited by our markers, or multiple lines with our markers as prefix and suffix.
				// If we are here, we know the line begins with our marker header, but is there additional data on the line?
				if(readLine.length() > sMarker.length()) {
					MachineReading mr = new MachineReading(readLine.substring(sMarker.length(),readLine.length()));
					mb.add(mr);
				}
				while( !(readLine = marlinLines.takeFirst()).startsWith(sMarker) ) {
						if( readLine == null || readLine.length() == 0 ) {
							//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
							break;
						}
						// Is our delimiting marker part of a one-line payload, or used at the end of a multiline payload?
						if(readLine.endsWith(sMarker)) {
							MachineReading mr = new MachineReading(readLine.substring(0,readLine.length()-sMarker.length()));
							mb.add(mr);
							break;
						}
						//if( Props.DEBUG ) System.out.println(readLine);
						MachineReading mr = new MachineReading(readLine);
						mb.add(mr);
				}
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}

			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}	
		});
		//
		// No checksum
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.NOLINECHECK.val());			
		topics.put(topicNames.NOLINECHECK.val(), new TopicList(this, topicNames.NOLINECHECK.val(),16) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				String sMarker = MSG_BEGIN+topicNames.NOLINECHECK.val()+MSG_TERMINATE;
				// Account for payloads on one line, delimited by our markers, or multiple lines with our markers as prefix and suffix.
				// If we are here, we know the line begins with our marker header, but is there additional data on the line?
				if(readLine.length() > sMarker.length()) {
					MachineReading mr = new MachineReading(readLine.substring(sMarker.length(),readLine.length()));
					mb.add(mr);
				}
				while( !(readLine = marlinLines.takeFirst()).startsWith(sMarker) ) {
						if( readLine == null || readLine.length() == 0 ) {
							//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
							break;
						}
						// Is our delimiting marker part of a one-line payload, or used at the end of a multiline payload?
						if(readLine.endsWith(sMarker)) {
							MachineReading mr = new MachineReading(readLine.substring(0,readLine.length()-sMarker.length()));
							mb.add(mr);
							break;
						}
						//if( Props.DEBUG ) System.out.println(readLine);
						MachineReading mr = new MachineReading(readLine);
						mb.add(mr);
				}
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}

			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}	
		});
		//
		// Checksum mismatch
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.CHECKMISMATCH.val());			
		topics.put(topicNames.CHECKMISMATCH.val(), new TopicList(this, topicNames.CHECKMISMATCH.val(),16) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				String sMarker = MSG_BEGIN+topicNames.CHECKMISMATCH.val()+MSG_TERMINATE;
				// Account for payloads on one line, delimited by our markers, or multiple lines with our markers as prefix and suffix.
				// If we are here, we know the line begins with our marker header, but is there additional data on the line?
				if(readLine.length() > sMarker.length()) {
					MachineReading mr = new MachineReading(readLine.substring(sMarker.length(),readLine.length()));
					mb.add(mr);
				}
				while( !(readLine = marlinLines.takeFirst()).startsWith(sMarker) ) {
						if( readLine == null || readLine.length() == 0 ) {
							//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
							break;
						}
						// Is our delimiting marker part of a one-line payload, or used at the end of a multiline payload?
						if(readLine.endsWith(sMarker)) {
							MachineReading mr = new MachineReading(readLine.substring(0,readLine.length()-sMarker.length()));
							mb.add(mr);
							break;
						}
						//if( Props.DEBUG ) System.out.println(readLine);
						MachineReading mr = new MachineReading(readLine);
						mb.add(mr);
				}
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}

			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}	
		});
		//
		// Line sequence out of order
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.LINESEQ.val());			
		topics.put(topicNames.LINESEQ.val(), new TopicList(this, topicNames.LINESEQ.val(),16) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				String sMarker = MSG_BEGIN+topicNames.LINESEQ.val()+MSG_TERMINATE;
				// Account for payloads on one line, delimited by our markers, or multiple lines with our markers as prefix and suffix.
				// If we are here, we know the line begins with our marker header, but is there additional data on the line?
				if(readLine.length() > sMarker.length()) {
					MachineReading mr = new MachineReading(readLine.substring(sMarker.length(),readLine.length()));
					mb.add(mr);
				}
				while( !(readLine = marlinLines.takeFirst()).startsWith(sMarker) ) {
						if( readLine == null || readLine.length() == 0 ) {
							//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
							break;
						}
						// Is our delimiting marker part of a one-line payload, or used at the end of a multiline payload?
						if(readLine.endsWith(sMarker)) {
							MachineReading mr = new MachineReading(readLine.substring(0,readLine.length()-sMarker.length()));
							mb.add(mr);
							break;
						}
						//if( Props.DEBUG ) System.out.println(readLine);
						MachineReading mr = new MachineReading(readLine);
						mb.add(mr);
				}
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}

			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}	
		});
		//
		// M115 report
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M115.val());			
		topics.put(topicNames.M115.val(), new TopicList(this, topicNames.M115.val(),16) {
			@Override
			public void retrieveData(String readLine) throws InterruptedException {
				String sMarker = MSG_BEGIN+topicNames.M115.val()+MSG_TERMINATE;
				// Account for payloads on one line, delimited by our markers, or multiple lines with our markers as prefix and suffix.
				// If we are here, we know the line begins with our marker header, but is there additional data on the line?
				if(readLine.length() > sMarker.length()) {
					MachineReading mr = new MachineReading(readLine.substring(sMarker.length(),readLine.length()));
					mb.add(mr);
				}
				while( !(readLine = marlinLines.takeFirst()).startsWith(sMarker) ) {
						if( readLine == null || readLine.length() == 0 ) {
							//if(Props.DEBUG)System.out.println("Empty line returned from readLine");
							break;
						}
						// Is our delimiting marker part of a one-line payload, or used at the end of a multiline payload?
						if(readLine.endsWith(sMarker)) {
							MachineReading mr = new MachineReading(readLine.substring(0,readLine.length()-sMarker.length()));
							mb.add(mr);
							break;
						}
						//if( Props.DEBUG ) System.out.println(readLine);
						MachineReading mr = new MachineReading(readLine);
						mb.add(mr);
				}
				mb.add(MachineReading.EMPTYREADING);
				synchronized(AsynchDemuxer.mutexWrite) {
					AsynchDemuxer.mutexWrite.notifyAll();
				}
			}

			@Override
			public Object getResult(MachineReading mr) {
				return mr.getReadingValString();
			}	
		});
		
		// spin the main loop to read lines from the Marlinspike and muxx them
		ThreadPoolManager.getInstance().spin(this, "SYSTEM");

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
    				System.out.println("AsynchDemux Cannot convert double value from:"+rnum);
    			}
    		}
    	}
    	System.out.println("AsynchDemux Can't get valid Double value from:"+readLine);
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
      				System.out.println("AsynchDemux Cannot convert integer value from:"+rnum);
      			}
      		}
      	}
      	System.out.println("AsynchDemux Can't get valid Integer value from:"+readLine);
      	return 0;
	}
      
    public synchronized String getReadingValueString(String readLine) {
      	if( readLine != null ) {
      		int sindex = readLine.indexOf(" ");
      		if( sindex != -1 && sindex+1 < readLine.length() ) {
      			return readLine.substring(sindex+1);
      		}
      	}
      	System.out.println("AsynchDemux Can't get valid String from raw line:"+readLine);
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
      					System.out.println("AsynchDemux Cannot convert Integer from:"+rnum);
      				}
      			}
	       	}	
	       	System.out.println("AsynchDemux Can't get valid reading number from:"+readLine);
	       	return 0;
	}

 
	/**
	 * Configure the robot with a series of G-code directives at startup in file startup.gcode
	 * @throws IOException
	 */
	public synchronized void config() throws IOException {
		// now read the startup G-code directives to initiate
		//String[] starts = FileIOUtilities.readAllLines("", "startup.gcode", ";");
		List<String> starts = FileIOUtilities.getConfig();
		for(String s : starts) {
			System.out.println("Startup GCode:"+s);
			addWrite(this,s);
		}
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
		// spin another worker thread to take queued write requests and send them on to the Marlinspike
		// Take requests from write queue and send them to the serial port of marlinspike. Wait for the same
		// response as request to be ack from our corresponding retrieveData with a notifyAll on mutexWrite.
		ThreadPoolManager.getInstance().spin(new Runnable() {
			@Override
			public void run() {
				try {
					while(shouldRun) {
						String writeReq = takeWrite();
						dataPort.writeLine(writeReq);
						synchronized(AsynchDemuxer.mutexWrite) {
							try {
								AsynchDemuxer.mutexWrite.wait(500);
							} catch (InterruptedException e) {
								System.out.println("AsynchDemux Timeout - No write response from Marlinspike for:"+writeReq);
								e.printStackTrace();
							}
						}
					}
				} catch (IOException e) {
					e.printStackTrace();
				}
			}	
		});
		// spin another worker thread to take Marlinspike lines from circular blocking deque and demux them
		ThreadPoolManager.getInstance().spin(new Runnable() {
			String line,fop;
			char op;
			@Override
			public void run() {
				while(shouldRun) {
					int endDelim = -1;
					try {
						line = marlinLines.takeFirst();
						fop = parseDirective(line);
						if( fop == null || fop.length() == 0 ) {
							System.out.println("AsynchDemux Cannot demux directive from line:"+line);
							continue;
						}
					} catch(InterruptedException e) {
						shouldRun = false;
						break;
					}
					try {
						TopicList tl = topics.get(fop);
						if( tl != null ) {
							tl.retrieveData(line);
						} else {
							System.out.println("AsynchDemux Cannot retrieve topic "+fop+" from raw directive for line:"+line);
							continue;
						}			
					} catch(IndexOutOfBoundsException ioob) {
						System.out.println("AsynchDemux Empty or malformed directive from line:"+line);
						continue;
					} catch (InterruptedException e) {
						shouldRun = false;
						break;
					}
				}
			}
		});	
		isRunning = true;
		while(shouldRun) {
			String line = dataPort.readLine();
			boolean overwrite = marlinLines.add(line);
			if(overwrite)
				System.out.println("AsynchDemux WARNING - INBOUND MARLINSPIKE QUEUE OVERWRITE!");
			if(DEBUG)
				System.out.println(this.getClass().getName()+" main read loop readLine:"+line);
		} // shouldRun
		isRunning = false;
	}
	
	private boolean isLineTerminal(String line) {
		int endDelim;
		String fop;
		try {
			if((line.charAt(0)) == '<' ) {
				// has to equal one of > or />
				endDelim = line.indexOf(MSG_TERMINATE);
				if(endDelim == -1)
					endDelim = line.indexOf('>');
				else
					return true;
				if( endDelim == -1 ) {
					return false;
				}		
				fop = line.substring(1, endDelim);
				if(fop.startsWith("/"))
					return true;
				if(fop.endsWith("/"))
					return true;
				//if(DEBUG)
				//	System.out.println("op:"+op);
			}
			return false;
		} catch (IndexOutOfBoundsException ioob) {
			return false;
		}
	}
	
	private boolean isLineTerminal(String line, String directive) {
		int endDelim;
		String fop;
		try {
			if((line.charAt(0)) == '<' ) {
				// has to equal one of > or />
				endDelim = line.indexOf(MSG_TERMINATE);
				if(endDelim == -1) {
					endDelim = line.indexOf('>');
				} else {
					if(line.contains(directive))
						return true;
					else
						return false;
				}
				if( endDelim == -1 ) {
					return false;
				}		
				fop = line.substring(1, endDelim);
				if(fop.startsWith("/"))
					if(line.contains(directive))
						return true;
					else
						return false;
				if(fop.endsWith("/"))
					if(line.contains(directive))
						return true;
					else
						return false;
				//if(DEBUG)
				//	System.out.println("op:"+op);
			}
			return false;
		} catch (IndexOutOfBoundsException ioob) {
			return false;
		}
	}
	
	private String parseDirective(String line) {
		int endDelim = -1;
		String fop;
		try {
			if(line.charAt(0) == '<' ) {
				// has to equal one of > or />
				endDelim = line.indexOf(MSG_TERMINATE);
				if(endDelim == -1)
					endDelim = line.indexOf('>');
				if( endDelim == -1 ) {
					return null;
				}	
				fop = line.substring(1, endDelim-1);
				if(fop.startsWith("/"))
					fop = fop.substring(1);
				if(fop.endsWith("/"))
					fop = fop.substring(0,fop.length()-1);
				if(fop.contains(" ")) {
					// see if we have a directive with a partial match
					Iterator<String> it = topics.keySet().iterator();
					while(it.hasNext()) {
						String directive = (String)it.next();
						endDelim = fop.indexOf(directive);
						if(endDelim != -1) {
							fop = fop.substring(endDelim, endDelim+directive.length()-1);
							return fop;
						}
					}
					return null;
				}
				return fop;
			}
		} catch (IndexOutOfBoundsException ioob) {
			return null;	
		}
		return null;
	}
	
	public String extractPayload(String line, String directive) {
		int endDelim = -1;
		String fop;
		try {
			if(line.charAt(0) == '<' ) {
				// has to equal one of > or />
				endDelim = line.indexOf(MSG_TERMINATE);
				if(endDelim == -1)
					endDelim = line.indexOf('>');
				if( endDelim == -1 ) {
					return null;
				}	
				fop = line.substring(1, endDelim-1);
				if(fop.startsWith("/"))
					fop = fop.substring(1);
				if(fop.endsWith("/"))
					fop = fop.substring(0,fop.length()-1);
				endDelim = fop.indexOf(directive);
				if(endDelim != -1) {
					fop = fop.substring(endDelim+directive.length()-1);
					return fop;
				}
				return fop;
			}
		} catch (IndexOutOfBoundsException ioob) {
			return null;	
		}
		return null;
	}
	
	public static void main(String[] args) throws Exception {
		// start demux
		AsynchDemuxer demuxer = new AsynchDemuxer();	
		demuxer.connect(ByteSerialDataPort.getInstance());
		// the L H and T values represent those to EXCLUDE
		// So we are looking for state 0 on digital pin and value not between L and H analog
		AsynchDemuxer.addWrite(demuxer,"M303 P54 L470 H510");
		AsynchDemuxer.addWrite(demuxer,"M303 P55 L470 H510");
		AsynchDemuxer.addWrite(demuxer,"M305 P30 T1");
		AsynchDemuxer.addWrite(demuxer,"M305 P46 T1");
		AsynchDemuxer.addWrite(demuxer,"M305 P47 T1");
		AsynchDemuxer.addWrite(demuxer,"M305 P49 T1");
	}

}
