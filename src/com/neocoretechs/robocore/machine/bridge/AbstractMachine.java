package com.neocoretechs.robocore.machine.bridge;

import java.io.IOException;

import com.neocoretechs.robocore.serialreader.DataPortInterface;


	/**
	 * Defines an abstraction of a type of machine ofmachineType that performs operations on a DataPortInterface
	 * @author jg
	 * Copyright  © 2012 Microcaliper Devices, LLC
	 */
    public abstract class AbstractMachine 
    {
        private String machineType;
  
        private String unitsOfMeasure=""; // metric/english    
        private DataPortInterface dataPort;
        
		public String getMachineType() {
			return machineType;
		}
		public void setMachineType(String machineType) {
			this.machineType = machineType;
		}
	
		public DataPortInterface getDataPort() {
			return dataPort;
		}
		public void setDataPort(DataPortInterface dataPort) {
			this.dataPort = dataPort;
		}

		public String getUnitsOfMeasure() {
			return unitsOfMeasure;
		}
		public void setUnitsOfMeasure(String units) {
			unitsOfMeasure = units;
		}
        public void close()
        {
            dataPort.close();
        }
        public void connect(boolean writeable) throws IOException {
        	dataPort.connect(writeable);
        }
 
        /**
         *  Wait a number of milliseconds via Thread.Sleep
         */
        public void waitMilliSecs(long miliSecs)
        {
        	try {
        		Thread.sleep(miliSecs);
        	} catch(InterruptedException ie) {}
        }

        public void write(String output) throws IOException
        {
            dataPort.writeLine(output);
        }

        public int read() throws IOException
        {
            return dataPort.read();
        }

        public String getPortName() { return dataPort.getPortName(); }
        
        public abstract void postProcess() throws Exception;
        
        public static double getReadingValueDouble(String readLine) {
        	if( readLine != null ) {
        		int sindex = readLine.indexOf(" ");
        		if( sindex != -1 && sindex+1 < readLine.length() ) {
        			String rnum = readLine.substring(sindex+1);
        			try {
        			return new Double(rnum).doubleValue();
        			} catch(Exception e) {
        				System.out.println("Bad reading from "+readLine);
        			}
        		}
        	}
        	System.out.println("Can't get valid value from acquired reading in "+readLine);
        	return 0;
		}

        public static int getReadingValueInt(String readLine) {
        	if( readLine != null ) {
        		int sindex = readLine.indexOf(" ");
        		if( sindex != -1 && sindex+1 < readLine.length() ) {
        			String rnum = readLine.substring(sindex+1);
        			try {
        			return new Integer(rnum).intValue();
        			} catch(Exception e) {
        				System.out.println("Bad reading from "+readLine);
        			}
        		}
        	}
        	System.out.println("Can't get valid value from acquired reading in "+readLine);
        	return 0;
		}
        
        public static String getReadingValueString(String readLine) {
        	if( readLine != null ) {
        		int sindex = readLine.indexOf(" ");
        		if( sindex != -1 && sindex+1 < readLine.length() ) {
        			String rnum = readLine.substring(sindex+1);
        			try {
        			return rnum;
        			} catch(Exception e) {
        				System.out.println("Bad reading from "+readLine);
        			}
        		}
        	}
        	System.out.println("Can't get valid value from acquired reading in "+readLine);
        	return null;
		}
        
		public static int getReadingNumber(String readLine) {
	       	if( readLine != null ) {
        		int sindex = readLine.indexOf(" ");
        		if( sindex != -1 && sindex+1 < readLine.length() ) {
        			String rnum = readLine.substring(0,sindex);
        			try {
        				return new Integer(rnum).intValue();
        			} catch(Exception e) {
        				System.out.println("Bad reading from "+readLine);
        				return 0;
        			}
        		}
        	}	
	       	System.out.println("Can't get valid sequence from acquired reading in "+readLine);
        	return 0;
		}

		@SuppressWarnings("unused")
		private String determineUnitsOfMeasure(String readLine) {
        	return "UNKNOWN";
		}

		protected String displayExceptionMessage1(/*Form statusForm*/)
        {        
           return "Either " + getPortName() + " is not attached or\n" +
                    "your " + getMachineType() + " is turned off\n" +
                    "Please correct and try again.";
        }
		
        public String stringSettings()
        {
            String msg = "Machine\n";
            msg = msg + getMachineType() + "\n";
            msg = msg + getDataPort().stringSettings();
            return msg;
        }
    }

