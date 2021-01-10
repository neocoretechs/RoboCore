package com.neocoretechs.robocore.machine.bridge;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

import com.neocoretechs.robocore.ThreadPoolManager;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
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
import com.neocoretechs.robocore.marlinspike.mcodes.status.eeprom;
import com.neocoretechs.robocore.marlinspike.mcodes.status.lineseq;
import com.neocoretechs.robocore.marlinspike.mcodes.status.motorcontrolSetting;
import com.neocoretechs.robocore.marlinspike.mcodes.status.motorfault;
import com.neocoretechs.robocore.marlinspike.mcodes.status.noMorG;
import com.neocoretechs.robocore.marlinspike.mcodes.status.nochecksum;
import com.neocoretechs.robocore.marlinspike.mcodes.status.nolinecheck;
import com.neocoretechs.robocore.marlinspike.mcodes.status.status;
import com.neocoretechs.robocore.marlinspike.mcodes.status.time;
import com.neocoretechs.robocore.marlinspike.mcodes.status.analogpin;
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
import com.neocoretechs.robocore.marlinspike.mcodes.status.unknownG;
import com.neocoretechs.robocore.marlinspike.mcodes.status.unknownM;
import com.neocoretechs.robocore.serialreader.ByteSerialDataPort;
import com.neocoretechs.robocore.serialreader.DataPortInterface;

/**
 * This class is the primary interface between real time data and the other subsystems.
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
 * This class is designed for horizontal scaling: multiple Marlinspike boards can be attached to different ports, and
 * an instance of this class can be created for each {@see MegaControl}.
 * @author Jonathan Groff (C) NeoCoreTechs 2019,2020,2021
 *
 */
public class AsynchDemuxer implements Runnable {
	private static boolean DEBUG = true;
	private volatile boolean shouldRun = true;
	private volatile boolean isRunning = false;
	private DataPortInterface dataPort;
	public Object mutexWrite = new Object();
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
		EEPROM("eeprom"),
		G4("G4"),G5("G5"),G99("G99"),G100("G100"),
		M0("M0"),M1("M1"),M2("M2"),M3("M3"),M4("M4"),M5("M5"),M6("M6"),M7("M7"),M8("M8"),M9("M9"),M10("M10"),M11("M11"),M12("M12"),
		M33("M33"),M35("M35"),M36("M36"),M37("M37"),M38("M38"),M39("M39"),M40("M40"),M41("M41"),M42("M42"),M45("M45"),M47("M47"),
		M80("M80"),M81("M81"),M301("M301"),M302("M302"),M304("M304"),M306("M306"),
		M445("M445"),M500("M500"),M501("M501"),M502("M502"),M799("M799"),M999("M999"),
		M115("FIRMWARE_NAME:Marlinspike RoboCore"); // followed by FIRMWARE_URL,PROTOCOL_VERSION,MACHINE_TYPE,MACHINE NAME,MACHINE_UUID
		String name;
		topicNames(String name) { this.name = name;}
		public String val() { return name; }
	};
	
	private Map<String, TopicList> topics = new ConcurrentHashMap<String, TopicList>(topicNames.values().length);
	public TopicListInterface getTopic(String group) { return topics.get(group); }
	
	private CircularBlockingDeque<String> marlinLines = new CircularBlockingDeque<String>(1024);
	public void clearLineBuffer() { marlinLines.clear(); }
	private CircularBlockingDeque<String> toWrite = new CircularBlockingDeque<String>(1024);
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
	
	public CircularBlockingDeque<String> getMarlinLines() { return marlinLines;}
	
	public synchronized void connect(DataPortInterface dataPort) throws IOException {
		this.dataPort = dataPort;
		dataPort.connect(true);
	}
	
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
		ThreadPoolManager.getInstance().spin(new G4(this, topics), topicNames.G4.val());
		//
		// G5
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.G5.val());
		ThreadPoolManager.getInstance().spin(new G5(this, topics), topicNames.G5.val());
		//
		// G99
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.G99.val());
		ThreadPoolManager.getInstance().spin(new G99(this, topics), topicNames.G99.val());
		//
		// G100
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.G100.val());
		ThreadPoolManager.getInstance().spin(new G100(this, topics), topicNames.G100.val());
		//
		// M0
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M0.val());
		ThreadPoolManager.getInstance().spin(new M0(this, topics), topicNames.M0.val());
		//
		// M1
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M1.val());
		ThreadPoolManager.getInstance().spin(new M1(this, topics), topicNames.M1.val());
		//
		// M2
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M2.val());
		ThreadPoolManager.getInstance().spin(new M2(this, topics), topicNames.M2.val());
		//
		// M3
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M3.val());
		ThreadPoolManager.getInstance().spin(new M3(this, topics), topicNames.M3.val());
		//
		// M4
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M4.val());
		ThreadPoolManager.getInstance().spin(new M4(this, topics), topicNames.M4.val());
		//
		// M5
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M5.val());
		ThreadPoolManager.getInstance().spin(new M5(this, topics), topicNames.M5.val());
		//
		// M6
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M6.val());
		ThreadPoolManager.getInstance().spin(new M6(this, topics), topicNames.M6.val());
		//
		// M7
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M7.val());
		ThreadPoolManager.getInstance().spin(new M7(this, topics), topicNames.M7.val());
		//
		// M8
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M8.val());
		ThreadPoolManager.getInstance().spin(new M8(this, topics), topicNames.M8.val());
		//
		// M9
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M9.val());
		ThreadPoolManager.getInstance().spin(new M9(this, topics), topicNames.M9.val());
		//
		// M10
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M10.val());
		ThreadPoolManager.getInstance().spin(new M10(this, topics), topicNames.M10.val());
		//
		// M101
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M11.val());
		ThreadPoolManager.getInstance().spin(new M11(this, topics), topicNames.M11.val());
		//
		// M12
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M12.val());
		ThreadPoolManager.getInstance().spin(new M12(this, topics), topicNames.M12.val());
		//
		// M33
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M33.val());
		ThreadPoolManager.getInstance().spin(new M33(this, topics), topicNames.M33.val());
		//
		// M35
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M35.val());
		ThreadPoolManager.getInstance().spin(new M35(this, topics), topicNames.M35.val());
		//
		// M36
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M36.val());
		ThreadPoolManager.getInstance().spin(new M36(this, topics), topicNames.M36.val());
		//
		// M37
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M37.val());
		ThreadPoolManager.getInstance().spin(new M37(this, topics), topicNames.M37.val());
		//
		// M38
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M38.val());
		ThreadPoolManager.getInstance().spin(new M38(this, topics), topicNames.M38.val());
		//
		// M39
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M39.val());
		ThreadPoolManager.getInstance().spin(new M39(this, topics), topicNames.M39.val());
		//
		// M40
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M40.val());
		ThreadPoolManager.getInstance().spin(new M40(this, topics), topicNames.M40.val());
		//
		// M41
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M41.val());
		ThreadPoolManager.getInstance().spin(new M41(this, topics), topicNames.M41.val());
		//
		// M42
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M42.val());
		ThreadPoolManager.getInstance().spin(new M42(this, topics), topicNames.M42.val());
		//
		// M44 - report on digitalpin
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.DIGITALPIN.val());
		ThreadPoolManager.getInstance().spin(new digitalpin(this, topics), topicNames.DIGITALPIN.val());
		//
		// M45
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M45.val());
		ThreadPoolManager.getInstance().spin(new M45(this, topics), topicNames.M45.val());
		//
		// M46 - report on analogpin
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.ANALOGPIN.val());
		ThreadPoolManager.getInstance().spin(new analogpin(this, topics), topicNames.ANALOGPIN.val());
		//
		// M47
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M47.val());
		ThreadPoolManager.getInstance().spin(new M47(this, topics), topicNames.M47.val());
		//
		// M80
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M80.val());
		ThreadPoolManager.getInstance().spin(new M80(this, topics), topicNames.M80.val());
	
		//
		// M81
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M81.val());
		ThreadPoolManager.getInstance().spin(new M81(this, topics), topicNames.M81.val());
		//
		// M301
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M301.val());
		ThreadPoolManager.getInstance().spin(new M301(this, topics), topicNames.M301.val());
		//
		// M302
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M302.val());
		ThreadPoolManager.getInstance().spin(new M302(this, topics), topicNames.M302.val());
		//
		// M304
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M304.val());
		ThreadPoolManager.getInstance().spin(new M304(this, topics), topicNames.M304.val());
		//
		// M306
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M306.val());
		ThreadPoolManager.getInstance().spin(new M306(this, topics), topicNames.M306.val());

		//
		// M445
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M445.val());
		ThreadPoolManager.getInstance().spin(new M445(this, topics), topicNames.M445.val());
		//
		// M500
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M500.val());
		ThreadPoolManager.getInstance().spin(new M500(this, topics), topicNames.M500.val());
		//
		// M501
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M501.val());
		ThreadPoolManager.getInstance().spin(new M501(this, topics), topicNames.M501.val());
		//
		// M502
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M502.val());
		ThreadPoolManager.getInstance().spin(new M502(this, topics), topicNames.M502.val());
		//
		// EEPROM (M503 response)
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.EEPROM.val());
		ThreadPoolManager.getInstance().spin(new eeprom(this, topics), topicNames.EEPROM.val());
		//
		// M799
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M799.val());
		ThreadPoolManager.getInstance().spin(new M799(this, topics), topicNames.M799.val());
		//
		// M999 - reset Marlinspike, issue command, 15ms delay, then suicide
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M999.val());
		ThreadPoolManager.getInstance().spin(new M999(this, topics), topicNames.M999.val());
		//
		// status - M700
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.STATUS.val());
		ThreadPoolManager.getInstance().spin(new status(this, topics), topicNames.STATUS.val());
		//
		// Dataset
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.DATASET.val());
		ThreadPoolManager.getInstance().spin(new dataset(this, topics), topicNames.DATASET.val());

		//
		// Battery
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.BATTERY.val());
		ThreadPoolManager.getInstance().spin(new battery(this, topics), topicNames.BATTERY.val());
		//
		// Motorfault
		//                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        
		if(DEBUG) 
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.MOTORFAULT.val());
		ThreadPoolManager.getInstance().spin(new motorfault(this, topics), topicNames.MOTORFAULT.val());
		//
		// Ultrasonic
		//
		if(DEBUG) 
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.ULTRASONIC.val());
		ThreadPoolManager.getInstance().spin(new ultrasonic(this, topics), topicNames.ULTRASONIC.val());

		//
		// reporting functions...
		// Assigned pins
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.ASSIGNEDPINS.val());
		ThreadPoolManager.getInstance().spin(new assignedPins(this, topics), topicNames.ASSIGNEDPINS.val());
		//
		// Motorcontrol
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.MOTORCONTROLSETTING.val());	
		ThreadPoolManager.getInstance().spin(new motorcontrolSetting(this, topics), topicNames.MOTORCONTROLSETTING.val());
		//
		// PWM control
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.PWMCONTROLSETTING.val());	
		ThreadPoolManager.getInstance().spin(new PWMcontrolsetting(this, topics), topicNames.PWMCONTROLSETTING.val());
		//
		// Controller status
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.CONTROLLERSTATUS.val());			
		ThreadPoolManager.getInstance().spin(new controllerStatus(this, topics), topicNames.CONTROLLERSTATUS.val());
		//
		// time
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.TIME.val());			
		ThreadPoolManager.getInstance().spin(new time(this, topics), topicNames.TIME.val());
		//
		// Controller stopped
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.CONTROLLERSTOPPED.val());			
		ThreadPoolManager.getInstance().spin(new controllerStopped(this, topics), topicNames.CONTROLLERSTOPPED.val());
		//
		// No M or G code
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.NOMORGCODE.val());			
		ThreadPoolManager.getInstance().spin(new noMorG(this, topics), topicNames.NOMORGCODE.val());
		//
		// Bad motor
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.BADMOTOR.val());			
		ThreadPoolManager.getInstance().spin(new badmotor(this, topics), topicNames.BADMOTOR.val());
		//
		// Bad PWM
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.BADPWM.val());			
		ThreadPoolManager.getInstance().spin(new badPWM(this, topics), topicNames.BADPWM.val());
		//
		// Unknown G code
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.UNKNOWNG.val());			
		ThreadPoolManager.getInstance().spin(new unknownG(this, topics), topicNames.UNKNOWNG.val());
		//
		// Unknown M code
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.UNKNOWNM.val());			
		ThreadPoolManager.getInstance().spin(new unknownM(this, topics), topicNames.UNKNOWNM.val());
		//
		// Bad Control
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.BADCONTROL.val());			
		ThreadPoolManager.getInstance().spin(new badcontrol(this, topics), topicNames.BADCONTROL.val());
		//
		// No checksum
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.NOCHECKSUM.val());			
		ThreadPoolManager.getInstance().spin(new nochecksum(this, topics), topicNames.NOCHECKSUM.val());

		//
		// No line check
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.NOLINECHECK.val());			
		ThreadPoolManager.getInstance().spin(new nolinecheck(this, topics), topicNames.NOLINECHECK.val());

		//
		// Checksum mismatch
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.CHECKMISMATCH.val());			
		ThreadPoolManager.getInstance().spin(new checkmismatch(this, topics), topicNames.CHECKMISMATCH.val());
		//
		// Line sequence out of order
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.LINESEQ.val());			
		ThreadPoolManager.getInstance().spin(new lineseq(this, topics), topicNames.LINESEQ.val());
		//
		// M115 report
		//
		if(DEBUG)
			System.out.println("AsynchDemuxer.Init bring up "+topicNames.M115.val());
		ThreadPoolManager.getInstance().spin(new M115(this, topics), topicNames.M115.val());
	
		
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
						synchronized(mutexWrite) {
							try {
								// the Marlinspike is a single threaded harvard microcontroller, if it
								// takes more than 500 ms to do something, its a major bottleneck to the whole system.
								mutexWrite.wait(500);
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
							System.out.println("AsynchDemux Cannot demux directive from line:"+line);
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
							ArrayList<String> payload = new ArrayList<String>();
							String data = marlinLines.takeFirst();
							String sload = parseDirective(data);
							payload.add(data);
							while(sload != null && sload.length() > 0 && !(isLineTerminal(sload) && sload.equals(fop))) {
								if(DEBUG)
									System.out.println(this.getClass().getName()+":"+sload);
								data = marlinLines.takeFirst();
								sload = parseDirective(data);
								payload.add(data);
							}	
							TopicList tl = topics.get(fop);
							if( tl != null ) {
								if(DEBUG)
									System.out.println("AsynchDemux call out to topic:"+tl.mb.getGroup());
								tl.retrieveData(payload);
							} else {
								System.out.println("AsynchDemux Cannot retrieve topic "+fop+" from raw directive for line:"+line);
								continue;
							}
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
