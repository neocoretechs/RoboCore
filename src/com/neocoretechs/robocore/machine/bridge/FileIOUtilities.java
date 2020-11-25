package com.neocoretechs.robocore.machine.bridge;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.DataInputStream;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.text.DateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;


/**
* Copyright NeoCoreTechs (C) 2020
* @author Jonathan Groff 
*/
public class FileIOUtilities {
		private static boolean DEBUG = true;
		private static final String propsFile = "startup.gcode";
		private static String propfile = null;
		private static ArrayList<String> config = new ArrayList<String>();
		/**
		 * assume properties file is in 'startup.gcode'
		 */
		static {
			try {
				String file = System.getProperty(propsFile);
				if (file == null) {
					init(top());
				} else {
					if(DEBUG)
						System.out.println("Loading properties:"+file);
					init(new FileInputStream(file));
				}
			} catch (IOException ioe) {
				throw new RuntimeException(ioe.toString());
			}
		}
        static DateFormat f = DateFormat.getDateInstance(DateFormat.SHORT, Locale.ENGLISH);
        public static final String dataDirectory = "/home/pi/"; // this is our extended partition mount on debian, our bogus dirs on Win for dev
        public static final String portSettingsFile = "rcportsettings.cfg";
 
        /**
    	 * Load the properties from the stream
    	 * @param propFile The stream for reading properties
    	 * @exception IOException If the read fails
    	 */
    	public static void init(InputStream propFile) throws IOException {
    		try {							   	
                BufferedReader reader =  new BufferedReader(new InputStreamReader(propFile));
                String line = null;
                while ((line = reader.readLine()) != null) {
                	//System.out.println("Startup :"+line);
                    if( !line.startsWith(";")) {
                    	config.add(line);
                    }
                }
                if(DEBUG)
                	System.out.println(f.getCalendar().getTime()+":"+propFile+" read with "+config.size()+" lines");
                reader.close();
    		} catch (Exception ex) {
    			throw new IOException("FATAL ERROR:  unable to load "+propsFile+" file " + ex.toString());
    		}
    	}
    	
    	public static List<String> getConfig() { return config; }
    	
    	/**
    	 * Find the top level resource for props
    	 * @return the InputStream of property resource
    	 * @exception IOException if we can't get the resource
    	 */
    	public static InputStream top() throws IOException {
    		java.net.URL loader =
    			ClassLoader.getSystemResource(propsFile);
    		if (loader == null) {
    			propfile = System.getProperty(propsFile); // now we look for -Dstartup.gcode= on cmdl
    			if( propfile == null ) {
    				propfile = dataDirectory+propsFile;
    			}
    			loader = ClassLoader.getSystemResource(propfile);
    		}
    		if( DEBUG )
    			System.out.println("Loading properties:"+loader);
    		return loader.openStream();
    	}

        /**
         * 
         * @param lstFile
         * @throws IOException
         */
        public static void writePortSettings() throws IOException
        {
            String fileName = dataDirectory + portSettingsFile;
            FileWriter lstFile = new FileWriter(fileName);
            lstFile.write("[Mega2560]\r\n");
            lstFile.write("Port=/dev/ttyACM0\r\n");
            lstFile.write("PortSettings=115200,8,n,1\r\n");
            lstFile.flush();
            lstFile.close();
        }

        public static String[] readAllLines(String filePath, String fileName) throws IOException {
        	FileReader fr = new FileReader(dataDirectory + filePath + fileName);
            BufferedReader bufferedReader = new BufferedReader(fr);
            List<String> lines = new ArrayList<String>();
            String line = null;
            while ((line = bufferedReader.readLine()) != null) {
                lines.add(line);
            }
            bufferedReader.close();
            return lines.toArray(new String[lines.size()]);
        }
  
        public static void writeAllLines(String filePath, String fileName, String[] lines) throws IOException {
        	FileWriter fw = new FileWriter(dataDirectory + filePath + fileName);
            BufferedWriter bufferedWriter = new BufferedWriter(fw);
            for(String line : lines) {
                bufferedWriter.write(line);
            }
            bufferedWriter.flush();
            bufferedWriter.close();
            fw.flush();
            fw.close();
        }
        
        public static String[] readAllLines(String filePath, String fileName, String commentLineDelim) throws IOException {
        	FileReader fr = new FileReader(dataDirectory + filePath + fileName);
            BufferedReader bufferedReader = new BufferedReader(fr);
            List<String> lines = new ArrayList<String>();
            String line = null;
            while ((line = bufferedReader.readLine()) != null) {
            	//System.out.println("Startup :"+line);
                if( !line.startsWith(commentLineDelim)) {
                	lines.add(line);
                }
            }
            bufferedReader.close();
            if(DEBUG )
            	System.out.println(f.getCalendar().getTime()+":"+(dataDirectory + filePath + fileName)+" read with "+lines.size()+" lines");
            return lines.toArray(new String[lines.size()]);
        }
        
        public static List<List<String>> readToList(String filePath, String fileName, String delim) throws IOException { 	
        	BufferedReader input =  new BufferedReader(new FileReader(dataDirectory + fileName));
            List<List<String>> mainContainer = new ArrayList<List<String>>();
        	String line = null;
        	while (( line = input.readLine()) != null)
        	{
        		String[] data = line.split(delim);
                List<String> lines = new ArrayList<String>();
        		lines.addAll(Arrays.asList(data));
        		mainContainer.add(lines);
        	}
        	input.close();
        	return mainContainer;
        }
        
		public static List<List<String>> readToList(String filePath, String fileName) throws IOException {
            BufferedReader bufferedReader = new BufferedReader(new FileReader(dataDirectory + filePath));
            List<List<String>> mainContainer = new ArrayList<List<String>>();
            String line = null;
            while ((line = bufferedReader.readLine()) != null) {
                List<String> lines = new ArrayList<String>();
                lines.add(line);
                mainContainer.add(lines);
            }
            bufferedReader.close();
            return mainContainer;
		}
		
		public static String read(String fileName) throws IOException {
	          BufferedReader bufferedReader = new BufferedReader(new FileReader(dataDirectory + fileName));
	          StringBuffer line = new StringBuffer();
	          String in;
	          while ( (in = bufferedReader.readLine()) != null) {
	        	   line.append(in);
	          }
	          bufferedReader.close();
	          return line.toString();
		}
		
		public static void write(String fileName, String load) throws IOException {
			FileWriter fw = new FileWriter(dataDirectory+fileName);
			fw.write(load);
			fw.flush();
			fw.close();
		}
		
		public static void writeFromList(String string, List<List<String>> hTable) {
			// TODO Auto-generated method stub
			
		}
}
