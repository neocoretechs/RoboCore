package com.neocoretechs.robocore.services;

import java.io.BufferedReader;
import java.io.FileReader;

import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;

public class fileReader<T> implements Runnable {

		public volatile boolean shouldRun = true;
		private CircularBlockingDeque<T> pubdataPWM;
		String path;
		Class<T> clazz;
		public fileReader(CircularBlockingDeque<T> cbd, String path, Class<T> clazz) {
			this.path = path;
			this.pubdataPWM = cbd;
			this.clazz = clazz;
		}

		@Override
		public void run() {
			while(shouldRun) {
			try {
				FileReader fis = new FileReader(path);
				BufferedReader br = new BufferedReader(fis);
				String s = br.readLine();
				br.close();
				fis.close();
				System.out.println(s);
				String left = s.substring(0,s.indexOf(","));
				String right = s.substring(s.indexOf(",")+1);
				System.out.println(left+","+right);
				pubdataPWM.addLast(setValue(left));
				pubdataPWM.addLast(setValue(right));
				Thread.sleep(5);
			} catch (Exception e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			}
		}
	
		@SuppressWarnings("unchecked")
	    public T setValue(String input) {
			T value;
	        if (clazz.isAssignableFrom(String.class)) {
	            value = (T) input;
	        } else if (clazz.isAssignableFrom(Integer.class)) {
	            value = (T) Integer.valueOf(input);
	        } else if (clazz.isAssignableFrom(Boolean.class)) {
	            value = (T) Boolean.valueOf(input);
	        } else if (clazz.isAssignableFrom(Double.class)) {
	            value = (T) Double.valueOf(input);
	        } else {
	            throw new IllegalArgumentException("Bad type.");
	        }
	        return value;
	    }
		
}
