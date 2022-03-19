package com.neocoretechs.robocore.marlinspike;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.concurrent.BrokenBarrierException;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CyclicBarrier;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.neocoretechs.robocore.SynchronizedFixedThreadPoolManager;
import com.neocoretechs.robocore.config.Robot;
import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;
import com.neocoretechs.robocore.machine.bridge.FileIOUtilities;
import com.neocoretechs.robocore.machine.bridge.MachineBridge;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.TopicListInterface;

import com.neocoretechs.robocore.marlinspike.gcodes.G100;
import com.neocoretechs.robocore.marlinspike.gcodes.G4;
import com.neocoretechs.robocore.marlinspike.gcodes.G5;
import com.neocoretechs.robocore.marlinspike.gcodes.G99;
import com.neocoretechs.robocore.marlinspike.mcodes.M0;
import com.neocoretechs.robocore.marlinspike.mcodes.M1;
import com.neocoretechs.robocore.marlinspike.mcodes.M10;
import com.neocoretechs.robocore.marlinspike.mcodes.M11;
import com.neocoretechs.robocore.marlinspike.mcodes.M12;
import com.neocoretechs.robocore.marlinspike.mcodes.M2;
import com.neocoretechs.robocore.marlinspike.mcodes.M3;
import com.neocoretechs.robocore.marlinspike.mcodes.M301;
import com.neocoretechs.robocore.marlinspike.mcodes.M302;
import com.neocoretechs.robocore.marlinspike.mcodes.M304;
import com.neocoretechs.robocore.marlinspike.mcodes.M306;
import com.neocoretechs.robocore.marlinspike.mcodes.M33;
import com.neocoretechs.robocore.marlinspike.mcodes.M35;
import com.neocoretechs.robocore.marlinspike.mcodes.M36;
import com.neocoretechs.robocore.marlinspike.mcodes.M37;
import com.neocoretechs.robocore.marlinspike.mcodes.M38;
import com.neocoretechs.robocore.marlinspike.mcodes.M39;
import com.neocoretechs.robocore.marlinspike.mcodes.M4;
import com.neocoretechs.robocore.marlinspike.mcodes.M40;
import com.neocoretechs.robocore.marlinspike.mcodes.M41;
import com.neocoretechs.robocore.marlinspike.mcodes.M42;
import com.neocoretechs.robocore.marlinspike.mcodes.M445;
import com.neocoretechs.robocore.marlinspike.mcodes.M45;
import com.neocoretechs.robocore.marlinspike.mcodes.M47;
import com.neocoretechs.robocore.marlinspike.mcodes.M5;
import com.neocoretechs.robocore.marlinspike.mcodes.M500;
import com.neocoretechs.robocore.marlinspike.mcodes.M501;
import com.neocoretechs.robocore.marlinspike.mcodes.M502;
import com.neocoretechs.robocore.marlinspike.mcodes.M6;
import com.neocoretechs.robocore.marlinspike.mcodes.M7;
import com.neocoretechs.robocore.marlinspike.mcodes.M799;
import com.neocoretechs.robocore.marlinspike.mcodes.M8;
import com.neocoretechs.robocore.marlinspike.mcodes.M80;
import com.neocoretechs.robocore.marlinspike.mcodes.M81;
import com.neocoretechs.robocore.marlinspike.mcodes.M9;
import com.neocoretechs.robocore.marlinspike.mcodes.M999;
import com.neocoretechs.robocore.marlinspike.mcodes.status.M115;
import com.neocoretechs.robocore.marlinspike.mcodes.status.PWMcontrolsetting;
import com.neocoretechs.robocore.marlinspike.mcodes.status.digitalpin;
import com.neocoretechs.robocore.marlinspike.mcodes.status.digitalpinsetting;
import com.neocoretechs.robocore.marlinspike.mcodes.status.eeprom;
import com.neocoretechs.robocore.marlinspike.mcodes.status.lineseq;
import com.neocoretechs.robocore.marlinspike.mcodes.status.motorcontrolSetting;
import com.neocoretechs.robocore.marlinspike.mcodes.status.motorfault;
import com.neocoretechs.robocore.marlinspike.mcodes.status.noMorG;
import com.neocoretechs.robocore.marlinspike.mcodes.status.nochecksum;
import com.neocoretechs.robocore.marlinspike.mcodes.status.nolinecheck;
import com.neocoretechs.robocore.marlinspike.mcodes.status.pwmpinsetting;
import com.neocoretechs.robocore.marlinspike.mcodes.status.status;
import com.neocoretechs.robocore.marlinspike.mcodes.status.time;
import com.neocoretechs.robocore.marlinspike.mcodes.status.analogpin;
import com.neocoretechs.robocore.marlinspike.mcodes.status.analogpinsetting;
import com.neocoretechs.robocore.marlinspike.mcodes.status.assignedPins;
import com.neocoretechs.robocore.marlinspike.mcodes.status.badPWM;
import com.neocoretechs.robocore.marlinspike.mcodes.status.badcontrol;
import com.neocoretechs.robocore.marlinspike.mcodes.status.badmotor;
import com.neocoretechs.robocore.marlinspike.mcodes.status.battery;
import com.neocoretechs.robocore.marlinspike.mcodes.status.checkmismatch;
import com.neocoretechs.robocore.marlinspike.mcodes.status.controllerStatus;
import com.neocoretechs.robocore.marlinspike.mcodes.status.controllerStopped;
import com.neocoretechs.robocore.marlinspike.mcodes.status.dataset;
import com.neocoretechs.robocore.marlinspike.mcodes.status.ultrasonic;
import com.neocoretechs.robocore.marlinspike.mcodes.status.ultrasonicpinsetting;
import com.neocoretechs.robocore.marlinspike.mcodes.status.unknownG;
import com.neocoretechs.robocore.marlinspike.mcodes.status.unknownM;

import com.neocoretechs.robocore.serialreader.DataPortInterface;

/**
 * This class is the primary interface between real time data and the other subsystems.<p/>
 * Its primary function is to demultiplex the input stream coming from {@code DataPortInterface} 
 * data sources such as the attached Marlinspike enabled microcontroller, I.e. Mega2560 etc, that implement the interface and 
 * utilize a protocol with '<header>',line number, data value.
 * Each 'topic' described by the demultiplexed header as it flows in with its associated data is expected to have:
 * OPTIONAL:<br/>
 * 1) A thread that services the final processing listener class and deque for the given topic '<header>' <br/>
 * 2) A listener class that is serviced by the above thread that takes raw MachineReadings and transforms them if necessary <br/>
 * MANDATORY:<br/>
 * 3) An instance of {@code DataPortInterface} to connect to. <br/>
 * 4) A 'TopicList' class in the hash table with its associated 'header' <br/>
 * 5) An instance of 'MachineBridge' that operates on the raw data for a given topic 'header' <br/>
 * The optional items are necessary for data streamed at high rates.
 * As the various topics are demuxxed by the thread servicing this class, the 'retrieveData' for each 'TopicList' 
 * is invoked to place the 'MachineReading' element in the deque associated with the MachineBridge for that topic.
 * The listener waits for a take from that particular MachineBridge and massages the data to be placed in its own deque
 * in the format and with the proper additions and exclusions from the raw MachineReading.
 * When an element in the listener is present and ready for a 'take' from that deque the item is considered ready for use
 * in the system.<p/>
 * The size of each listener circular deque is determined during invocation of the 'init' method of the MachineBridge for that topic.
 * This demuxxer runs in its own thread as well such that it may operate unimpeded while the listeners can take their time
 * processing the data. In this way a near realtime response is preserved.<p/>
 * The pipeline to start this service is as follows:<br/>
 * <code>connect(DataPortInterface)</code><br/>
 * <code>init()</code><br/>
 * <code>config(List<String> of String Marlinspike commands)</code><br/>
 * This class is designed for horizontal scaling: multiple Marlinspike boards can be attached to different ports, and
 * an instance of this class can be created for each {@link MarlinspikeControl}.
 * @author Jonathan Groff (C) NeoCoreTechs 2019,2020,2021
 *
 */
public class AsynchDemuxer implements Runnable {
	public static boolean DEBUG = false;
	private static boolean PORTDEBUG = true;
	private volatile boolean shouldRun = true;
	private DataPortInterface dataPort;
	public CyclicBarrier mutexWrite = new CyclicBarrier(2);
	private CircularBlockingDeque<String> marlinLines = new CircularBlockingDeque<String>(1024);
	private CircularBlockingDeque<String> toWrite = new CircularBlockingDeque<String>(1024);
	private MarlinspikeManager marlinSpikeManager;
	private final static String MSG_BEGIN = "<";
	private final static String MSG_TERMINATE ="/>";
	protected long RESPONSE_WAIT_MS = 2000; // Number of MS to wait for response from Marlinspike port

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
		DIGITALPINSETTING("digitalpinsetting"),
		ANALOGPINSETTING("analogpinsetting"),
		ULTRASONICPINSETTING("ultrasonicpinsetting"),
		PWMPINSETTING("pwmpinsetting"),
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
		EEPROM("eeprom"),
		G4("G4"),G5("G5"),G99("G99"),G100("G100"),
		M0("M0"),M1("M1"),M2("M2"),M3("M3"),M4("M4"),M5("M5"),M6("M6"),M7("M7"),M8("M8"),M9("M9"),M10("M10"),M11("M11"),M12("M12"),M13("M13"),
		M33("M33"),M35("M35"),M36("M36"),M37("M37"),M38("M38"),M39("M39"),M40("M40"),M41("M41"),M42("M42"),M45("M45"),M47("M47"),
		M80("M80"),M81("M81"),M301("M301"),M302("M302"),M304("M304"),M306("M306"),
		M445("M445"),M500("M500"),M501("M501"),M502("M502"),M799("M799"),M999("M999"),
		M115("FIRMWARE_NAME:Marlinspike RoboCore"); // followed by FIRMWARE_URL,PROTOCOL_VERSION,MACHINE_TYPE,MACHINE NAME,MACHINE_UUID
		String name;
		topicNames(String name) { this.name = name;}
		public String val() { return name; }
	};
	
	private Map<String, TopicListInterface> topics = new ConcurrentHashMap<String, TopicListInterface>(topicNames.values().length);

	public AsynchDemuxer(MarlinspikeManager marlinspikeManager) { 
		this.marlinSpikeManager = marlinspikeManager;
	}
	
	public TopicListInterface getTopic(String group) { return topics.get(group); }
	
	public synchronized void clearLineBuffer() { marlinLines.clear(); }
	public synchronized void clearWriteBuffer() { toWrite.clear(); }
	
	public synchronized TypeSlotChannelEnable getNameToTypeSlotChannel(String name) {
		try {
			return marlinSpikeManager.getTypeSlotChannelEnableByName(name);
		} catch(NoSuchElementException npe) {
			throw new RuntimeException(npe);
		}
	}
	
	/**
	 * Add a write request to the outbound queue. The queue is circular and blocking and technically, a deque.
	 * If the elements reach a predetermined upper bound they are emplaced at the beginning. If this occurs,
	 * a warning message is displayed but otherwise operation is unaffected. If these warnings are an issue,
	 * the size of the deque must be increased.
	 * @param ad The AsynchDemuxer to queue to.
	 * @param req The request to be enqueued.
	 */
	public static void addWrite(AsynchDemuxer ad, String req) {
		boolean overwrite;
		synchronized(ad) {
			overwrite = ad.toWrite.addLast(req);
		}
		if(overwrite)
			System.out.println("WARNING - OUTBOUND MARLINSPIKE QUEUE OVERWRITE!");
	}

	public MachineBridge getMachineBridge(String group) {
		return topics.get(group).getMachineBridge();
	}
	
	public CircularBlockingDeque<String> getMarlinLines() { return marlinLines;}
	
	public synchronized void connect(DataPortInterface dataPort) throws IOException {
		this.dataPort = dataPort;
		dataPort.connect(true);
		init();
	}
	/**
	 * Initialize the topic names for this AsynchDemuxer
	 */
	private synchronized void init() {
		// initialize the fixed thread pool manager
		SynchronizedFixedThreadPoolManager.init(3, Integer.MAX_VALUE, new String[]{dataPort.getPortName()});
		//
		// G4
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.G4.val());
		topics.put(topicNames.G4.val(), new G4(this).getTopicList());
		//
		// G5
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.G5.val());
		topics.put(topicNames.G5.val(),new G5(this).getTopicList());
		//
		// G99
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.G99.val());
		topics.put(topicNames.G99.val(),new G99(this).getTopicList());
		//
		// G100
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.G100.val());
		topics.put(topicNames.G100.val(), new G100(this).getTopicList());
		//
		// M0
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M0.val());
		topics.put(topicNames.M0.val(), new M0(this).getTopicList());
		//
		// M1
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M1.val());
		topics.put(topicNames.M1.val(), new M1(this).getTopicList());
		//
		// M2
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M2.val());
		topics.put(topicNames.M2.val(), new M2(this).getTopicList());
		//
		// M3
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M3.val());
		topics.put(topicNames.M3.val(), new M3(this).getTopicList());
		//
		// M4
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M4.val());
		topics.put(topicNames.M4.val(), new M4(this).getTopicList());
		//
		// M5
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M5.val());
		topics.put(topicNames.M5.val(), new M4(this).getTopicList());
		//
		// M6
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M6.val());
		topics.put( topicNames.M6.val(), new M6(this).getTopicList());
		//
		// M7
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M7.val());
		topics.put(topicNames.M7.val(), new M7(this).getTopicList());
		//
		// M8
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M8.val());
		topics.put(topicNames.M8.val(), new M8(this).getTopicList());
		//
		// M9
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M9.val());
		topics.put(topicNames.M9.val(), new M9(this).getTopicList());
		//
		// M10
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M10.val());
		topics.put(topicNames.M10.val(), new M10(this).getTopicList());
		//
		// M101
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M11.val());
		topics.put(topicNames.M11.val(), new M11(this).getTopicList());
		//
		// M12
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M12.val());
		topics.put(topicNames.M12.val(), new M12(this).getTopicList());
		//
		// M13
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M13.val());
		topics.put(topicNames.M13.val(), new M12(this).getTopicList());
		//
		// M33
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M33.val());
		topics.put(topicNames.M33.val(), new M33(this).getTopicList());
		//
		// M35
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M35.val());
		topics.put(topicNames.M35.val(), new M35(this).getTopicList());
		//
		// M36
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M36.val());
		topics.put(topicNames.M36.val(), new M36(this).getTopicList());
		//
		// M37
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M37.val());
		topics.put(topicNames.M37.val(), new M37(this).getTopicList());
		//
		// M38
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M38.val());
		topics.put(topicNames.M38.val(), new M38(this).getTopicList());
		//
		// M39
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M39.val());
		topics.put(topicNames.M39.val(), new M39(this).getTopicList());
		//
		// M40
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M40.val());
		topics.put(topicNames.M40.val(), new M40(this).getTopicList());
		//
		// M41
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M41.val());
		topics.put(topicNames.M41.val(), new M41(this).getTopicList());
		//
		// M42
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M42.val());
		topics.put(topicNames.M42.val(), new M42(this).getTopicList());
		//
		// M44 - report on digitalpin
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.DIGITALPIN.val());
		topics.put(topicNames.DIGITALPIN.val(), new digitalpin(this).getTopicList());
		//
		// M45
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M45.val());
		topics.put(topicNames.M45.val(), new M45(this).getTopicList());
		//
		// M46 - report on analogpin
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.ANALOGPIN.val());
		topics.put(topicNames.ANALOGPIN.val(), new analogpin(this).getTopicList());
		//
		// M47
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M47.val());
		topics.put(topicNames.M47.val(), new M47(this).getTopicList());
		//
		// M80
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M80.val());
		topics.put(topicNames.M80.val(), new M80(this).getTopicList());
	
		//
		// M81
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M81.val());
		topics.put(topicNames.M81.val(), new M81(this).getTopicList());
		//
		// M301
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M301.val());
		topics.put(topicNames.M301.val(), new M301(this).getTopicList());
		//
		// M302
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M302.val());
		topics.put(topicNames.M302.val(), new M302(this).getTopicList());
		//
		// M304
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M304.val());
		topics.put(topicNames.M304.val(), new M304(this).getTopicList());
		//
		// M306
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M306.val());
		topics.put(topicNames.M306.val(), new M306(this).getTopicList());

		//
		// M445
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M445.val());
		topics.put(topicNames.M445.val(), new M445(this).getTopicList());
		//
		// M500
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M500.val());
		topics.put(topicNames.M500.val(), new M500(this).getTopicList());
		//
		// M501
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M501.val());
		topics.put(topicNames.M501.val(), new M501(this).getTopicList() );
		//
		// M502
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M502.val());
		topics.put(topicNames.M502.val(), new M502(this).getTopicList());
		//
		// EEPROM (M503 response)
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.EEPROM.val());
		topics.put(topicNames.EEPROM.val(), new eeprom(this).getTopicList());
		//
		// M799
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M799.val());
		topics.put(topicNames.M799.val(), new M799(this).getTopicList());
		//
		// M999 - reset Marlinspike, issue command, 15ms delay, then suicide
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M999.val());
		topics.put(topicNames.M999.val(), new M999(this).getTopicList());
		//
		// status - M700
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.STATUS.val());
		topics.put(topicNames.STATUS.val(), new status(this).getTopicList());
		//
		// Dataset
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.DATASET.val());
		topics.put(topicNames.DATASET.val(), new dataset(this).getTopicList());

		//
		// Battery
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.BATTERY.val());
		topics.put(topicNames.BATTERY.val(), new battery(this).getTopicList());
		//
		// Motorfault
		//                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
		if(DEBUG) 
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.MOTORFAULT.val());
		topics.put(topicNames.MOTORFAULT.val(), new motorfault(this).getTopicList());
		//
		// Ultrasonic
		//
		if(DEBUG) 
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.ULTRASONIC.val());
		topics.put(topicNames.ULTRASONIC.val(), new ultrasonic(this).getTopicList());

		//
		// reporting functions...
		// Assigned pins
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.ASSIGNEDPINS.val());
		topics.put(topicNames.ASSIGNEDPINS.val(), new assignedPins(this).getTopicList());
		//
		// Motorcontrol
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.MOTORCONTROLSETTING.val());	
		topics.put(topicNames.MOTORCONTROLSETTING.val(), new motorcontrolSetting(this).getTopicList());
		//
		// PWM control
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.PWMCONTROLSETTING.val());	
		topics.put(topicNames.PWMCONTROLSETTING.val(), new PWMcontrolsetting(this).getTopicList());
		//
		// Controller status
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.CONTROLLERSTATUS.val());			
		topics.put(topicNames.CONTROLLERSTATUS.val(), new controllerStatus(this).getTopicList());
		//
		// time
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.TIME.val());			
		topics.put(topicNames.TIME.val(), new time(this).getTopicList());
		//
		// Controller stopped
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.CONTROLLERSTOPPED.val());			
		topics.put(topicNames.CONTROLLERSTOPPED.val(), new controllerStopped(this).getTopicList());
		//
		// No M or G code
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.NOMORGCODE.val());			
		topics.put(topicNames.NOMORGCODE.val(), new noMorG(this).getTopicList());
		//
		// Bad motor
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.BADMOTOR.val());			
		topics.put(topicNames.BADMOTOR.val(), new badmotor(this).getTopicList());
		//
		// Bad PWM
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.BADPWM.val());			
		topics.put(topicNames.BADPWM.val(), new badPWM(this).getTopicList());
		//
		// Unknown G code
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.UNKNOWNG.val());			
		topics.put(topicNames.UNKNOWNG.val(), new unknownG(this).getTopicList());
		//
		// Unknown M code
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.UNKNOWNM.val());			
		topics.put(topicNames.UNKNOWNM.val(), new unknownM(this).getTopicList());
		//
		// Bad Control
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.BADCONTROL.val());			
		topics.put(topicNames.BADCONTROL.val(), new badcontrol(this).getTopicList());
		//
		// No checksum
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.NOCHECKSUM.val());			
		topics.put(topicNames.NOCHECKSUM.val(), new nochecksum(this).getTopicList());

		//
		// No line check
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.NOLINECHECK.val());			
		topics.put(topicNames.NOLINECHECK.val(), new nolinecheck(this).getTopicList());

		//
		// Checksum mismatch
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.CHECKMISMATCH.val());			
		topics.put(topicNames.CHECKMISMATCH.val(), new checkmismatch(this).getTopicList());
		//
		// Line sequence out of order
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.LINESEQ.val());			
		topics.put(topicNames.LINESEQ.val(), new lineseq(this).getTopicList());
		//
		// M115 report
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M115.val());
		topics.put(topicNames.M115.val(), new M115(this).getTopicList());
		//
		// M701 report
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.DIGITALPINSETTING.val());
		topics.put(topicNames.DIGITALPINSETTING.val(), new digitalpinsetting(this).getTopicList());
		//
		// M702 report
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.ANALOGPINSETTING.val());
		topics.put(topicNames.ANALOGPINSETTING.val(), new analogpinsetting(this).getTopicList());
		//
		// M703 report
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.ULTRASONICPINSETTING.val());
		topics.put(topicNames.ULTRASONICPINSETTING.val(), new ultrasonicpinsetting(this).getTopicList());
		//
		// M704 report
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.PWMPINSETTING.val());
		topics.put(topicNames.PWMPINSETTING.val(), new pwmpinsetting(this).getTopicList());
		
		// spin the main loop to read lines from the Marlinspike and muxx them
		SynchronizedFixedThreadPoolManager.spin(this, dataPort.getPortName());

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
	 * Configure the robot with a series of G-code directives at startup in file startup.gcode.
	 * Wait until write queue empties and number of waiters on write is 0
	 * @throws IOException
	 */
	public synchronized void config() throws IOException {
		// now read the startup G-code directives to initiate
		//String[] starts = FileIOUtilities.readAllLines("", "startup.gcode", ";");
		List<String> starts = FileIOUtilities.getConfig();
		config(starts);
	}
	
	public synchronized void config(List<String> starts) throws IOException {
		for(String s : starts) {
			System.out.printf("%s Thread:%s Port:%s Startup GCode:%s%n",this.getClass().getName(),Thread.currentThread().getName(),dataPort.getPortName(),s);
			addWrite(this,s);
		}
		while(!toWrite.isEmpty() || mutexWrite.getNumberWaiting() > 0)
			try {
				Thread.sleep(0,10);
			} catch (InterruptedException e) {
				throw new IOException(e);
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
		// Take requests from write queue and send them to the serial port of marlinspike or the queue of
		// the waiting MarlinspikeDataPort thread. Wait for the same
		// response as request to be ack from our corresponding retrieveData with a barrier synch on mutexWrite.
		SynchronizedFixedThreadPoolManager.spin(new Runnable() {
			@Override
			public void run() {
				String writeReq = null;
				try {
					while(shouldRun) {
						try {
							writeReq = toWrite.takeFirst();
							if(DEBUG)
								System.out.println(this.getClass().getName()+" "+Thread.currentThread().getName()+" writeLine:"+writeReq);
							if(writeReq == null || writeReq.isEmpty())
								continue;
							dataPort.writeLine(writeReq);
							mutexWrite.await(RESPONSE_WAIT_MS, TimeUnit.MILLISECONDS);
							if(DEBUG)
								System.out.println(this.getClass().getName()+" "+Thread.currentThread().getName()+" await done:"+writeReq);
						} catch (TimeoutException e) {
							if(DEBUG || PORTDEBUG)
								System.out.println(this.getClass().getName()+" "+Thread.currentThread().getName()+
										" NO RESPONSE FROM PORT "+dataPort.getPortName()+" IN TIME FOR DIRECTIVE:"+writeReq);
						} finally {
							mutexWrite.reset();
							if(DEBUG)
								System.out.println(this.getClass().getName()+" "+Thread.currentThread().getName()+" reset");
						}
					}
				} catch (IOException | BrokenBarrierException | InterruptedException e) {
					e.printStackTrace();
				}
			}	
		}, dataPort.getPortName()); // transmit data to dataport thread
		
		// spin another worker thread to take Marlinspike lines from circular blocking deque and demux them.
		// this will process the responses from the dataport that have placed on the deque.
		SynchronizedFixedThreadPoolManager.spin(new Runnable() {
			String line,fop;
			@Override
			public void run() {
				while(shouldRun) {
					try {
						line = marlinLines.peekFirst();
						if(line == null) {
							Thread.sleep(1);
							continue;
						}
						if( line.length() == 0 ) {
							if(DEBUG || PORTDEBUG)
								System.out.println(this.getClass().getName()+" "+Thread.currentThread().getName()+
									" RESPONSE FROM PORT "+dataPort.getPortName()+" CANNOT DEMUX DIRECTIVE:"+line);
							// consume line
							marlinLines.takeFirst();
							continue;
						}
					} catch(InterruptedException e) {
						shouldRun = false;
						break;
					}
					try {
						fop = parseDirective(line);
						if(fop != null) {
							if(DEBUG)
								System.out.println("AsynchDemux Parsed directive:"+fop);
							TopicListInterface tl = topics.get(fop);
							if( tl != null ) {
								if(DEBUG)
									System.out.println("AsynchDemux call out to topic:"+tl.getMachineBridge().getGroup());
								ArrayList<String> payload = new ArrayList<String>();
								while(line != null) {
									line = marlinLines.takeFirst();
									payload.add(line);
									if(DEBUG)
										System.out.println(this.getClass().getName()+" "+tl+" payload:"+line);
									String sload = parseDirective(line);
									if(sload != null && sload.length() > 0 && isLineTerminal(line) && sload.equals(fop)) 
										break;
								}
								if(DEBUG)
									System.out.println(this.getClass().getName()+" payload size:"+payload.size()+" retrieveData:"+tl);
								tl.retrieveData(payload);
							} else {
								if(DEBUG || PORTDEBUG)
									System.out.println(this.getClass().getName()+" "+Thread.currentThread().getName()+
										" RESPONSE FROM PORT "+dataPort.getPortName()+" CANNOT RETRIEVE TOPIC:"+fop+" CANNOT DEMUX DIRECTIVE:"+line);
								// consume line
								marlinLines.takeFirst();
								continue;
							}
						}			
					} catch(IndexOutOfBoundsException ioob) {
						if(DEBUG || PORTDEBUG)
							System.out.println(this.getClass().getName()+" "+Thread.currentThread().getName()+
								" RESPONSE FROM PORT "+dataPort.getPortName()+" MALFORMED OR EMPTY LINE, CANNOT DEMUX DIRECTIVE:"+line);
						// consume line
						try {
							marlinLines.takeFirst();
						} catch (InterruptedException e) {}
						continue;
					} catch (InterruptedException e) {
						shouldRun = false;
						break;
					}
				}
			}
		},dataPort.getPortName());	// process data received from dataport thread

		// Read responses from dataport and add them to processing queue
		while(shouldRun) {
			try {
				String line = dataPort.readLine();
				boolean overwrite = marlinLines.add(line);
				if(overwrite)
					System.out.println(this.getClass().getName()+" "+Thread.currentThread().getName()+
						" RESPONSE FROM PORT "+dataPort.getPortName()+" WARNING - INBOUND MARLINSPIKE QUEUE OVERWRITE!");
				mutexWrite.await(RESPONSE_WAIT_MS, TimeUnit.MILLISECONDS);
				if(DEBUG)
					System.out.println(this.getClass().getName()+" "+Thread.currentThread().getName()+" await done:"+line);
			} catch (TimeoutException | InterruptedException | BrokenBarrierException e) {
				if(DEBUG || PORTDEBUG)
					System.out.println(this.getClass().getName()+" "+Thread.currentThread().getName()+
						" NO RESPONSE IN TIME FROM PORT "+dataPort.getPortName());
			} finally {
				mutexWrite.reset();
				if(DEBUG)
					System.out.println(this.getClass().getName()+" "+Thread.currentThread().getName()+" reset");
			}
		} // receive data from dataport thread

	}
	
	public boolean isLineTerminal(String line) {
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
	
	public String parseDirective(String line) {
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
				fop = line.substring(1, endDelim);
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
							fop = fop.substring(endDelim, endDelim+directive.length());
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
				fop = line.substring(1, endDelim);
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
		Robot r = new Robot();
		MarlinspikeManager mm = new MarlinspikeManager(r);
		AsynchDemuxer demuxer = new AsynchDemuxer(mm);
		/*
		demuxer.connect(ByteSerialDataPort.getInstance());
		// the L H and T values represent those to EXCLUDE
		// So we are looking for state 0 on digital pin and value not between L and H analog
		AsynchDemuxer.addWrite(demuxer,"M303 P54 L470 H510");
		AsynchDemuxer.addWrite(demuxer,"M303 P55 L470 H510");
		AsynchDemuxer.addWrite(demuxer,"M305 P30 T1");
		AsynchDemuxer.addWrite(demuxer,"M305 P46 T1");
		AsynchDemuxer.addWrite(demuxer,"M305 P47 T1");
		AsynchDemuxer.addWrite(demuxer,"M305 P49 T1");
		*/
		System.out.println(demuxer.parseDirective("<M0>\r\n"));
		System.out.println(demuxer.parseDirective("<M1>\r\n\r\n"));
		System.out.println(demuxer.parseDirective("<M11>\r"));
		System.out.println(demuxer.parseDirective("<M111>\r\r"));
		System.out.println(demuxer.parseDirective("<M9>\r\n"));
		System.out.println(demuxer.parseDirective("<M99>\r\n"));
		System.out.println(demuxer.parseDirective("<M999>\r\n"));
		System.out.println(demuxer.parseDirective("<M100>"));
		System.out.println(demuxer.parseDirective("<M0/>\r\n"));
		System.out.println(demuxer.parseDirective("<M1/>\r\n\r\n"));
		System.out.println(demuxer.parseDirective("<M11/>\r"));
		System.out.println(demuxer.parseDirective("<M111/>\r\r"));
		System.out.println(demuxer.parseDirective("<M9/>\r\n"));
		System.out.println(demuxer.parseDirective("<M99/>\r\n"));
		System.out.println(demuxer.parseDirective("<M999/>\r\n"));
		System.out.println(demuxer.parseDirective("<M100/>"));
	}

}
