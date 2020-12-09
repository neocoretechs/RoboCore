package com.neocoretechs.robocore.machine.bridge;


import java.io.StringReader;
import java.io.StringWriter;


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
 	 * @author jg
 	 * Copyright 2012,2017 NeoCoreTechs
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
    	CircularBlockingDeque<MachineReading> machineReadings = new CircularBlockingDeque<MachineReading>(256);
   
		private String group;
    	private static volatile MachineBridge instance = null;
    	public static MachineBridge getInstance(String group) {
    		if(instance == null) {
    			synchronized(MachineBridge.class) { 
    				if( instance == null ) {
    					instance = new MachineBridge(group);
    				}
    			}
    		}
    		return instance;
    	}
 
        public MachineBridge() { }
        public MachineBridge(String group) { this.group = group; }
        
        public CircularBlockingDeque<MachineReading> get() { return machineReadings; }
        
		
		public MachineReading take() {
				try {
					return machineReadings.takeFirst();
				} catch (InterruptedException e) {
					return null;
				}
		}
	
		public void add(MachineReading entry) {
				machineReadings.addLast(entry);
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
						return machineReadings.takeFirst();
					} catch (InterruptedException e) {
						return null; // premature end
					}
		}
		
		public synchronized static void fromXml(String xml) {
			//MachineBridge mb = MachineBridge.getInstance();
			StringReader reader = new StringReader(xml);
			JAXBContext context;
			try {
				context = JAXBContext.newInstance(MachineBridge.class, MachineReading.class, RawDataSubset.class);
		        //SchemaFactory sf = SchemaFactory.newInstance(XMLConstants.W3C_XML_SCHEMA_NS_URI); 
		        //Schema schema = sf.newSchema(new File("MachineReading.xsd")); 
				Unmarshaller m = context.createUnmarshaller();
		        //m.setSchema(schema);
				instance =  (MachineBridge)m.unmarshal(reader);
			} catch (JAXBException /*| SAXException*/ e) {
				e.printStackTrace();
			}
		}
		
		public synchronized static String toXml(String group) {
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
		
		@Override
		public String toString() {
			synchronized(MachineBridge.class) { 
				MachineReading mr;
				StringBuilder sb = new StringBuilder();
				while((mr = waitForNewReading()) != null) {
					sb.append(mr.toString());
				}
				return sb.toString();
			}
		}

    }

