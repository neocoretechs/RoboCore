package com.neocoretechs.robocore.machine.bridge;


import java.io.StringReader;
import java.io.StringWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Marshaller;
import javax.xml.bind.Unmarshaller;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlRootElement;
import javax.xml.bind.annotation.XmlType;
import javax.xml.bind.annotation.adapters.XmlJavaTypeAdapter;

import com.neocoretechs.robocore.ThreadPoolManager;


 	/**
 	 * Bridges machine ops and model implementation. The manner in which this acts as a bridge is that this is
 	 * a passive behavioral construct that acts in a thread safe manner in between two active threads; one
 	 * transmitting or placing the data, the other receiving or retrieving the data. After completion the
 	 * machineReadings ArrayList contains the data and can be accessed by other parts of the app in a thread safe manner
 	 * and will ignore the null in the array that signals the end for the listeners.
 	 * @author jg
 	 * Copyright © 2012 Microcaliper Devices, LLC
 	 */
	//@XmlRootElement @XmlType(factoryMethod="getInstance")
	@XmlRootElement(name="MachineBridge")
	@XmlAccessorType(XmlAccessType.FIELD)
    public final class MachineBridge
    {
		private static boolean DEBUG = true;
		//@XmlElement(name="machineReadings", type=MachineReading.class)
		@XmlJavaTypeAdapter(RawDataXmlAdapter.class)
		//@XmlElementWrapper()
		//@XmlAnyElement(lax=true)
    	List<MachineReading> machineReadings = new ArrayList<MachineReading>();
   
		private String group;
    	private static MachineBridge[] instance = null;
    	public static MachineBridge getInstance(String group) { 
    		if( instance == null ) {
    			instance = new MachineBridge[AsynchDemuxer.getTopicNames().length];
    			for(int i = 0; i < AsynchDemuxer.getTopicNames().length; i++) {
    				instance[i] = new MachineBridge(AsynchDemuxer.getTopicNames()[i]);
    			}
    		}
    		for(int i = 0; i < instance.length; i++)
    			if( instance[i].getGroup().equals(group) )
    				return instance[i];
    		return null;
    	}
 
        public MachineBridge() { }
        public MachineBridge(String group) { this.group = group; }
        
        public List<MachineReading> get() { return machineReadings; }
        
		public MachineReading get(int readNum) {
			synchronized(machineReadings) {
				return machineReadings.get(readNum);
			}
		}
		
		public MachineReading take() {
			synchronized(machineReadings) {
				return machineReadings.remove(0);
			}
		}
		public void set(int readNum, MachineReading entry) {
			synchronized(machineReadings) {
				machineReadings.add(readNum, entry);
			}
		}
		public void add(MachineReading entry) {
			synchronized(machineReadings) {
				machineReadings.add(entry);
				machineReadings.notifyAll();
			}
		}
		public void init() {
			synchronized(machineReadings) {
				machineReadings.clear();
			}
		}
		
		public String getGroup() { return group; }
		/**
		 * Wait for a new reading to arrive or get the next available one
		 * @param preVal The value to retrieve, updates this value to the next for re-use
		 * @return
		 */
		public MachineReading waitForNewReading(AtomicInteger preVal) {
			//if( Props.DEBUG)
			//	System.out.println("Bridge waiting..");
			synchronized(machineReadings) {
				if( preVal.get() == machineReadings.size() ) {
					try {
						machineReadings.wait();
					} catch (InterruptedException e) {
						return null; // premature end
					}
				}
				//if( Props.DEBUG)
				//	System.out.println("bridge wait exiting..");				
				return machineReadings.get(preVal.getAndIncrement());
			}
		}
		
		/**
		 * Wait with timeout
		 * @param millis
		 */
		public MachineReading waitForNewReading(AtomicInteger preVal, long millis) {
			if( DEBUG)
				System.out.println("Bridge waiting millis.."+millis);
			synchronized(machineReadings) {
				if( preVal.get() == machineReadings.size() ) {
					try {
						machineReadings.wait(millis);
					} catch (InterruptedException e) {}
				}
				if( DEBUG)
					System.out.println("bridge wait exiting..");
				return machineReadings.get(preVal.getAndIncrement());
			}
		}
		
		/**
		 * For external call functionality the null placed at the end to signal end will not be counted if present
		 * @return
		 */
		public int size() {
			synchronized(machineReadings)  {
				if( machineReadings.size() == 0)
					return 0;
				if( machineReadings.get(machineReadings.size()-1) == null )
					return machineReadings.size()-1;
				return machineReadings.size();
			}
		}
		
		public void waitForEnd() {
			synchronized(machineReadings) {
				while(true) {
					try {
						machineReadings.wait();
					} catch (InterruptedException e) {}
					if( machineReadings.get(machineReadings.size()-1) == null ) {
						return;
					}
				}
			}
		}
			
		public void signalEnd() {
			//if( Props.DEBUG )
			//	System.out.println("..End signal to Bridge");
			synchronized(machineReadings) {
				machineReadings.add(null);
				machineReadings.notifyAll();
			}
			ThreadPoolManager.getInstance().notifyGroup(group);	
			//if( Props.DEBUG)
			//	System.out.println("signalEnd exiting..");
		}
	
		public static void fromXml(String xml, String group) {
			//MachineBridge mb = MachineBridge.getInstance();
			StringReader reader = new StringReader(xml);
			JAXBContext context;
			try {
				context = JAXBContext.newInstance(MachineBridge.class, MachineReading.class, RawDataSubset.class);
		        //SchemaFactory sf = SchemaFactory.newInstance(XMLConstants.W3C_XML_SCHEMA_NS_URI); 
		        //Schema schema = sf.newSchema(new File("MachineReading.xsd")); 
				Unmarshaller m = context.createUnmarshaller();
		        //m.setSchema(schema);
				int i;
				for(i = 0; i < AsynchDemuxer.getTopicNames().length; i++ )
					if( AsynchDemuxer.getTopicNames()[i].equals(group) )
						break;
				instance[i] =  (MachineBridge)m.unmarshal(reader);
			} catch (JAXBException /*| SAXException*/ e) {
				e.printStackTrace();
			}
		}
		
		public static String toXml(String group) {
			MachineBridge mb = MachineBridge.getInstance(group);
			StringWriter writer = new StringWriter();
			JAXBContext context;
			try {
				context = JAXBContext.newInstance(MachineBridge.class, MachineReading.class, RawDataSubset.class);
				Marshaller m = context.createMarshaller();
				m.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, true);
				m.marshal(mb, writer);
			} catch (JAXBException e) {
				e.printStackTrace();
			}
			//if( Props.DEBUG ) System.out.println("Bridge: "+writer.toString());
			return writer.toString();
		}		

    }

