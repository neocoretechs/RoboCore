package com.neocoretechs.robocore.machine.bridge;

import java.io.StringReader;
import java.io.StringWriter;
import java.util.Iterator;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Marshaller;
import javax.xml.bind.Unmarshaller;
import javax.xml.bind.annotation.XmlAccessType;
import javax.xml.bind.annotation.XmlAccessorType;
import javax.xml.bind.annotation.XmlRootElement;
import javax.xml.bind.annotation.XmlType;
import javax.xml.bind.annotation.adapters.XmlJavaTypeAdapter;

 	/**
 	 * Bridges machine ops and model implementation. The manner in which this acts as a bridge is that this is
 	 * a passive behavioral construct that acts in a thread safe manner in between two active threads; one
 	 * transmitting or placing the data, the other receiving or retrieving the data. After completion the
 	 * machineReadings ArrayList contains the data and can be accessed by other parts of the app in a thread safe manner.
 	 * @author Jonathan Groff (C) Copyright 2012,2017 NeoCoreTechs
 	 */
	//@XmlRootElement @XmlType(factoryMethod="getInstance")
	@XmlRootElement(name="MachineBridge")
	@XmlAccessorType(XmlAccessType.FIELD)
    public final class MachineBridge {
		private static boolean DEBUG = false;
		//@XmlElement(name="machineReadings", type=MachineReading.class)
		@XmlJavaTypeAdapter(RawDataXmlAdapter.class)
		//@XmlElementWrapper()
		//@XmlAnyElement(lax=true)
    	CircularBlockingDeque<MachineReading> machineReadings = null;
   
		private String group;
 
        public MachineBridge() { }
        public MachineBridge(String group, int queueSize) {
         	this.group = group; 
        	this.machineReadings = new CircularBlockingDeque<MachineReading>(queueSize);
        }
        
        public CircularBlockingDeque<MachineReading> get() { return machineReadings; }
	
		public void add(MachineReading entry) {
			if(machineReadings.addLast(entry))
				if(DEBUG)
					System.out.println("WARNING: MachineBridge for "+group+" has overwritten its queue of size:"+machineReadings.length());
		}
		/**
		 * Initialize the blocking queue to receive data for the associated topic.
		 * The queue is a circular deque so the size must be chosen to represent
		 * the most current values but not so large as to deliver obsolete values
		 * 16 seems a good value for rapidly accumulating data.
		 * @param queuesize
		 */
		public void init(int queuesize) {
			machineReadings = new CircularBlockingDeque<MachineReading>(queuesize);		
		}
		
		public String getGroup() { return group; }
		/**
		 * Wait for a new reading to arrive or get the next available one
		 * @param preVal The value to retrieve, updates this value to the next for re-use
		 * @return
		 */
		public MachineReading waitForNewReading() {
					try {
						if(DEBUG)
							System.out.println(this.getClass().getName()+".waitForNewReading wait..");
						MachineReading mr = machineReadings.takeFirst();
						if(DEBUG)
							System.out.println(this.getClass().getName()+".waitForNewReading got "+mr);
						return mr;
					} catch (InterruptedException e) {
						return null; // premature end
					}
		}
		
		public synchronized static MachineBridge fromXml(String xml) throws JAXBException {
			//MachineBridge mb = MachineBridge.getInstance();
			StringReader reader = new StringReader(xml);
			JAXBContext context;	
			context = JAXBContext.newInstance(MachineBridge.class, MachineReading.class, RawDataSubset.class);
		    //SchemaFactory sf = SchemaFactory.newInstance(XMLConstants.W3C_XML_SCHEMA_NS_URI); 
		    //Schema schema = sf.newSchema(new File("MachineReading.xsd")); 
			Unmarshaller m = context.createUnmarshaller();
		    //m.setSchema(schema);
			return (MachineBridge)m.unmarshal(reader);

		}
		
		public synchronized static String toXml(MachineBridge mb) {
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
		
		@Override
		public String toString() {
			StringBuilder sb = new StringBuilder();
			synchronized(machineReadings.getMutex()) {
				Iterator<MachineReading> it = machineReadings.iterator();
				while(it.hasNext()) {
					sb.append(it.next());
					sb.append("\r\n");
				}
			return sb.toString();
			}
		}

    }

