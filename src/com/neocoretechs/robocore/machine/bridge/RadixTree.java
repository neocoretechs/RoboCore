package com.neocoretechs.robocore.machine.bridge;

import java.util.SortedMap;
import java.util.TreeMap;

/**
* Radix tree, or trie
* We are taking 2 16 bit ints and making a 32 bit linear key
* or 2 16 bit ints and a 32 bit payload and making a 64 bit key
* so the T type is constrained to int or long
* We are interchanging bits to do this al la Peano key for the 16 bit ints.
* @author Groff Copyright (C) NeoCoreTechs 2019
*/
public class RadixTree<T,V> {
	        public short radix = 1;
	        public short xoffset = 0;
	        public short yoffset = 0;
	        private TreeMap<T,V> treeMap;
	        /**
	         * Construct new Radix trie
	         * @param tradix magnitude (power of 10) by which to multiply x and y after adding offset, default 0
	         * @param txoffset amount to add to x before making key
	         * @param tyoffset amount to add to y before making key
	         */
	        public RadixTree(short tradix, short txoffset, short tyoffset) {
	                if( tradix > 0 ) radix = (short) Math.pow(10, tradix);
	                xoffset = txoffset;
	                yoffset = tyoffset;
	                treeMap = new TreeMap<T,V>();
	        }
	        /**
	         * Construct new Radix trie
	         * @param txoffset amount to add to x before making key
	         * @param tyoffset amount to add to y before making key
	         */
	        public RadixTree(short txoffset, short tyoffset) {
	                xoffset = txoffset;
	                yoffset = tyoffset;
	                treeMap = new TreeMap<T,V>();
	        }
	        /**
	         * Construct new Radix trie
	         */
	        public RadixTree() {
	                treeMap = new TreeMap<T,V>();
	        }
	        
	        public int makeKey(short x, short y) {
	                short xi = (short) ((x + xoffset) * radix);
	                short yi = (short) ((y + yoffset) * radix);
	                int kxy = 0; 

	                kxy += (xi >> 15) & 1;
	                kxy <<= 1;
	                kxy += (yi >> 15) & 1;

	                kxy <<= 1;
	                kxy += (xi >> 14) & 1;
	                kxy <<= 1;
	                kxy += (yi >> 14) & 1;

	                kxy <<= 1;
	                kxy += (xi >> 13) & 1;
	                kxy <<= 1;
	                kxy += (yi >> 13) & 1;

	                kxy <<= 1;
	                kxy += (xi >> 12) & 1;
	                kxy <<= 1;
	                kxy += (yi >> 12) & 1;

	                kxy <<= 1;
	                kxy += (xi >> 11) & 1;
	                kxy <<= 1;
	                kxy += (yi >> 11) & 1;

	                kxy <<= 1;
	                kxy += (xi >> 10) & 1;
	                kxy <<= 1;
	                kxy += (yi >> 10) & 1;

	                kxy <<= 1;
	                kxy += (xi >> 9) & 1;
	                kxy <<= 1;
	                kxy += (yi >> 9) & 1;

	                kxy <<= 1;
	                kxy += (xi >> 8) & 1;
	                kxy <<= 1;
	                kxy += (yi >> 8) & 1;

	                kxy <<= 1;
	                kxy += (xi >> 7) & 1;
	                kxy <<= 1;
	                kxy += (yi >> 7) & 1;

	                kxy <<= 1;
	                kxy += (xi >> 6) & 1;
	                kxy <<= 1;
	                kxy += (yi >> 6) & 1;

	                kxy <<= 1;
	                kxy += (xi >> 5) & 1;
	                kxy <<= 1;
	                kxy += (yi >> 5) & 1;

	                kxy <<= 1;
	                kxy += (xi >> 4) & 1;
	                kxy <<= 1;
	                kxy += (yi >> 4) & 1;

	                kxy <<= 1;
	                kxy += (xi >> 3) & 1;
	                kxy <<= 1;
	                kxy += (yi >> 3) & 1;

	                kxy <<= 1;
	                kxy += (xi >> 2) & 1;
	                kxy <<= 1;
	                kxy += (yi >> 2) & 1;

	                kxy <<= 1;
	                kxy += (xi >> 1) & 1;
	                kxy <<= 1;
	                kxy += (yi >> 1) & 1;

	                kxy <<= 1;
	                kxy += xi & 1;
	                kxy <<= 1;
	                kxy += yi & 1;

	                return kxy;
	    }

	    public V put(short x, short y, V tvalue) {       
	       int trtn = makeKey(x, y);
	       return treeMap.put((T)convertInstanceOfObject(trtn), tvalue);
	    }
	    public V put(short x, short y, int options, V tvalue) {       
		       int trtn = makeKey(x, y);
		       long ltrtn = (trtn << 32) | options;
		       return treeMap.put((T)convertInstanceOfObject(ltrtn), tvalue);
		}
	    public static <T> T convertInstanceOfObject(Object o) {
	        try {
	            return (T)o;
	        } catch(ClassCastException e) {
	        	System.out.println("Cant cast "+o.getClass()+" "+e);
	            return null;
	        }
	    }
	    /**
	    * Get range
	    */
	    public SortedMap<T,V> subMap(short x, short y, short tolerance) {
	       // flattened bit range, may lose some on bounds
	       // rtn.radixKey &= 0x7FFFFFFFFFF00000L;
	       // rtnHigh.radixKey = rtn.radixKey | 0xFFFFFL;
	       // we make a real range, but it costs a little
	       int rtn = 0;
	       if( x >= tolerance )
	    	   x -= tolerance;
	       if( y >= tolerance)
	    	   y -= tolerance;
	       makeKey((short)x,(short)y);
	       int rtnHigh = makeKey((short)(x+tolerance),(short)(y+tolerance));
	       if( rtn < 0 || rtnHigh < 0)
	    	   System.out.println("KEY OVERFLOW:"+x+" "+y+" tolerance:"+tolerance+" "+rtn+" "+rtnHigh);
//	       System.out.println(rtn.radixKey+" "+rtnHigh.radixKey);
	       return treeMap.subMap((T)convertInstanceOfObject(rtn), (T)convertInstanceOfObject(rtnHigh));
	    }
	    /**
	    * Get range with optional extra integer payload
	    * low range is x - tolerance, y - tolerance << 32 | lowOptions
	    * high is x + tolerance, y + tolerance << 32 | highOptions
	    * If you want guaranteed ranges use a 0 for lowOptions and a 0xFFFFFFFF for high
	    */
	    public SortedMap<T,V> subMap(short x, short y, int lowOptions, int highOptions, short tolerance) {
	       long rtn = 0;
		   if( x >= tolerance )
			   x -= tolerance;
		   if( y >= tolerance)
			   y -= tolerance;
	       rtn = (((long)makeKey(x,y)) << 32) | (long)lowOptions;
	       long rtnHigh = (((long)makeKey((short)(x+tolerance),(short)(y+tolerance))) << 32) | (long)highOptions;
	       if( rtn < 0 || rtnHigh < 0)
	    	   System.out.println("KEY OVERFLOW:"+x+" "+y+" tolerance:"+tolerance+" "+rtn+" "+rtnHigh);
//	       System.out.println(rtn.radixKey+" "+rtnHigh.radixKey);
	       return treeMap.subMap((T)convertInstanceOfObject(rtn), (T)convertInstanceOfObject(rtnHigh));
	    }
	
}
