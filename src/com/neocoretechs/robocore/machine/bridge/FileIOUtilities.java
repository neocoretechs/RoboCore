package com.neocoretechs.robocore.machine.bridge;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;


	/**
	 * Copyright Microcaliper Devices, LLC
	 * @author jg
	 *
	 */
	public class FileIOUtilities
	{
        static DateFormat f = DateFormat.getDateInstance(DateFormat.SHORT, Locale.ENGLISH);
        public static final String dataDirectory = "/home/pi/"; // this is our extended partition mount on debian, our bogus dirs on Win for dev
        public static final String portSettingsFile = "rcportsettings.cfg";
  
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
