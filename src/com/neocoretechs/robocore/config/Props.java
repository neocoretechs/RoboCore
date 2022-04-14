package com.neocoretechs.robocore.config;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Collection;
import java.util.Enumeration;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;
import java.util.Map.Entry;
import java.util.Properties;
import java.util.concurrent.ConcurrentHashMap;

/*
* Copyright (c) 2003, NeoCoreTechs
* All rights reserved.
* Redistribution and use in source and binary forms, with or without modification, 
* are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this list of
* conditions and the following disclaimer. 
* Redistributions in binary form must reproduce the above copyright notice, 
* this list of conditions and the following disclaimer in the documentation and/or
* other materials provided with the distribution. 
* Neither the name of NeoCoreTechs nor the names of its contributors may be 
* used to endorse or promote products derived from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
* PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR 
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
* TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED 
* OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/
/**
 * Load and retrieve from properties file for configuration purposes.
 * We assume the the properties file is in RoboCore.Properties which we try
 * to locate through system.getProperty("RoboCore.properties") and barring that,
 * attempt to load from the system resource stream.
 * Copyright NeoCoreTechs (C) 2020
 * @author Jonathan Groff
 */
public class Props {
	public static boolean DEBUG = false;
	private static final String propsFile = "RoboCore.properties";
	private static String propfile = null;
	public static String dataDirectory;
	public static Properties properties;
	/**
	 * assume properties file is in 'RoboCore.properties' defined at runtime by -DRoboCore.properties=file
	 * failing that, we will try to load a file name RoboCore.properties from the system resource stream
	 * located on the classpath, failing that, we will try to load RoboCore.properties from the dataDirectory path.
	 * If all else fails throw Runtime exception because we need this config.
	 */
	static {
		try {
			String file = System.getProperty(propsFile);
			properties = new Properties();
			if (file == null) {
				init(top());
			} else {
				File f = new File(file);
				if(f.exists())
					init(new FileInputStream(file));
				else
					init(top());
			}
		} catch (IOException ioe) {
			throw new RuntimeException(ioe.toString());
		}
	}
	
	/**
	 * Find the top level resource for props
	 * @return the InputStream of property resource
	 * @exception IOException if we can't get the resource
	 */
	public static InputStream top() throws IOException {
		java.net.URL loader =
			ClassLoader.getSystemResource(propsFile);
		if (loader == null) {
		   	dataDirectory = System.getProperty("jars.provision");
		   	if(dataDirectory == null)
		   		throw new IOException(propsFile+" cannot be loaded from any resource path to configure Robot");
	    	if(!dataDirectory.endsWith("/"))
	    			dataDirectory += "/";
			if( DEBUG )
				System.out.println("Loading properties:"+dataDirectory+propsFile);
			return new FileInputStream(dataDirectory+propsFile);
		}
		if( DEBUG )
			System.out.println("Loading properties:"+loader);
		return loader.openStream();
	}
	
	public static String getPropFile() {
		return propfile;
	}

	/**
	 * Load the properties from the stream
	 * @param propFile The stream for reading properties
	 * @exception IOException If the read fails
	 */
	public static void init(InputStream propFile) throws IOException {
		try {
			properties.load(propFile);
		} catch (Exception ex) {
			throw new IOException("FATAL ERROR:  unable to load "+propsFile+" file " + ex.toString());
		}
	}

	/**
	 * @param prop The property to retrieve
	 * @return The property as a boolean
	 * @exception IllegalArgumentException if not set. 
	 **/
	public static boolean toBoolean(String prop) {
		String val = Props.toString(prop);
		try {
			return Boolean.valueOf(val).booleanValue();
		} catch (Exception ex) {
			throw new IllegalArgumentException(
				"invalid value "
					+ val
					+ " for property "
					+ prop
					+ " (expected true/false)");
		}
	}

	/** 
	 * @param prop The property to retrieve
	 * @return The property as an int 
	 * @exception IllegalArgumentException if not set. 
	 **/
	public static int toInt(String prop) {
		String val = Props.toString(prop);
		try {
			return Integer.parseInt(val);
		} catch (NumberFormatException ex) {
			throw new IllegalArgumentException(
				"invalid value "
					+ val
					+ " for property "
					+ prop
					+ " (expected integer)");
		}
	}

	/** 
	 * @param prop The property to retrieve
	 * @return The property as a long 
	 * @exception IllegalArgumentException if not set. 
	 **/
	public static long toLong(String prop) {
		String val = Props.toString(prop); // can hurl
		try {
			return Long.parseLong(val);
		} catch (NumberFormatException ex) {
			throw new IllegalArgumentException(
				"invalid value "
					+ val
					+ " for property "
					+ prop
					+ " (expected long)");
		}
	}


	/** 
	 * @param prop The property to retrieve
	 * @return The property as a String 
	 * @exception IllegalArgumentException if not set. 
	 **/	
	public static String toString(String prop) {
		String result = properties.getProperty(prop);
		if (result == null)
			throw new IllegalArgumentException("property " + prop + " not set");
		if (result != null)
			result = result.trim();
		return result;
	}

	public static float toFloat(String prop) {
		String val = properties.getProperty(prop);
		try {
			return Float.valueOf(val).floatValue();
		} catch (Exception ex) {
			throw new IllegalArgumentException(
				"invalid value "
					+ val
					+ " for property "
					+ prop
					+ " (expected floating point)");
		}
	}
	/**
	 * Build collection of array parameters in the properties file.<p/>
	 * Three maps are used to build the eventual final structure.
	 * mainMap = ConcurrentHashMap<String, Map<Integer, Map<String, Object>>>
	 * subMap = ConcurrentHashMap<Integer, Map<String,Object>>
	 * propMap = ConcurrentHashMap<String,Object>
	 * formed from the properties:
	 * LUN[0].Channel:1
	 * LUN[1].Slot:0
	 * LUN[0].Controller:/dev/ttyACM0
	 * LUN[0].Slot:0
	 * LUN[1].Channel:1
	 * LUN[1].Controller:/dev/ttyACM1
	 * So mainMap has:
	 * <LUN, <0, <Channel, 1>>>
	 * <LUN, <1, <Channel, 1>>>
	 * <LUN, <0, <Slot, 0>>>
	 * <LUN, <1, <Slot, 0>>>
	 * <LUN, <0, <Controller, /dev/ttyACM0>>>
	 * <LUN, <1, <Controller, /dev/ttyACM1>>>
	 * @return
	 * @throws IllegalAccessException
	 */
	public static Map<String, Map<Integer, Map<String, Object>>> collectivizeProps() throws IllegalAccessException {
		Enumeration<Object> keys = properties.keys();
		Collection<Object> values = properties.values();
		Iterator<Object> vit = values.iterator();
		ConcurrentHashMap<String, Map<Integer, Map<String,Object>>> mainMap = new ConcurrentHashMap<String, Map<Integer, Map<String,Object>>>();
		while(keys.hasMoreElements()) {
			Object key = keys.nextElement();
			Object value = vit.next();
			int aindex = key.toString().indexOf("[");
			if(aindex != -1) {
				int bindex = key.toString().indexOf("]");
				if(bindex == -1)
					throw new IllegalAccessException("Bad array configuration on line:"+key+":"+value);
				// extract numerical value of collection element
				int xindex = Integer.parseInt(key.toString().substring(aindex+1,bindex));
				// extract name of collection mainMap param
				String pname = key.toString().substring(0,aindex);
				ConcurrentHashMap<Integer, Map<String,Object>> subMap = (ConcurrentHashMap<Integer, Map<String, Object>>) mainMap.get(pname);
				// extract name of collection element param
				String pename = key.toString().substring(bindex+2);
				if(subMap == null) {
						subMap = new ConcurrentHashMap<Integer, Map<String,Object>>();
						mainMap.put(pname, subMap);
				}
				if(DEBUG)
					System.out.println(pname+"["+xindex+"]."+pename+":"+value);
				Map<String,Object> propMap = subMap.get(xindex);
				if(propMap == null) {
					propMap = new ConcurrentHashMap<String,Object>();
					subMap.put(xindex, propMap);
				}
				propMap.put(pename,  value);
			} else {
				if(DEBUG)
					System.out.println("didnt find [ in "+key+" class:"+key.getClass().getName());
			}
		}
		return mainMap;
	}
	
	public static void main(String[] args) throws IOException {
		FileOutputStream fos;
		StringBuilder sb = new StringBuilder("<<MarlinSpike Configs>>\r\n");
		try {
			fos = new FileOutputStream(dataDirectory+"XML"+propsFile);
			properties.storeToXML(fos, "MarlinSpike Configs");
			try {
				Map<String, Map<Integer, Map<String, Object>>> globalConfigs = Props.collectivizeProps();
				Set<Entry<String, Map<Integer, Map<String, Object>>>> props = globalConfigs.entrySet();
				for(Entry<String, Map<Integer, Map<String, Object>>> e : props ) {
					sb.append(e.getKey());
					sb.append("\r\n");
					Map<Integer, Map<String, Object>> prop1 = e.getValue();
					Set<Integer> iset = prop1.keySet();
					Set<Entry<Integer, Map<String, Object>>> jset = prop1.entrySet();
					Iterator<Entry<Integer, Map<String, Object>>> it = jset.iterator();
					for(Integer i: iset) {
						sb.append("[");
						sb.append(i);
						sb.append("].");
						Entry<Integer, Map<String, Object>> elem = it.next();
						Map<String,Object> o = elem.getValue();
						Set<String> s = o.keySet();
						Collection<Object> c = o.values();
						Iterator<String> its = s.iterator();
						Iterator<Object> ito = c.iterator();
						while(its.hasNext()) {
							sb.append(its.next());
							sb.append(":");
							sb.append(ito.next());
							sb.append("\r\n");
						}
					}
						
				}
			} catch (IllegalAccessException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			System.out.println(sb.toString());
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		Robot r = new Robot();
		System.out.println(r.getName());
		System.out.println(r.toString());
	}

}
