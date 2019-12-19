package com.neocoretechs.robocore.machine.bridge;

import java.math.BigInteger;
import java.util.SortedMap;
import java.util.TreeMap;


/**
* Radix tree, or trie, a structure to index data in 2,3,4,5, and 6 dimensions.<br/>
* and letting us retrieve subsets based on varying windows of 2, 3, or 4 5 or 6 dimensions.<br/>
* We are taking 2 16 bit ints and making a 32 bit linear key,
* or 2 16 bit ints and a 32 bit payload and making a 64 bit key,
* OR 2 32 bit ints and making a long key!
* OR 4 16 bit shorts making a long key of 4 dimensions! (or 3 if you leave one zero).
* OR 6 16 bit shorts making a BigInteger key of 6 dimensions! (or 5). <br/>
* So what goes into the actual backing store TreeMap is an Integer, Long, or BigInteger key of
* interchanged bits that is a max of 96 bits in the BigInteger case. <br/>
* So the T type is constrained to int or long or BigInteger, because this is a radix tree.
* We are interchanging bits to do this al la Peano key for the component key values.<br/>
* Why? If we encode a 'flatened' or 'expanded' key, one which has the LSB set to all 0 ir 1, then we
* encode a multidimensional window of sorts into which collections can be aggregated and retrieved.<br/>
* When encoding points, we can specify the lower and upper bounds of a flattened and expended key set 
* to retrieve elements of a multidimensional set (see the subMap methods below), very fast.<br/>
* Noting the offsets in the constructors, for things to work properly in general we need to keep the keys
* in the positive cartesian coordinate space, since we are flattening bits and such.  
* @author Groff Copyright (C) NeoCoreTechs 2019
*/
public class RadixTree<T,V> {
		public static boolean DEBUG = false;
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
	        /**
	         * Generate a point key of max 16384 in x and y
	         * @param x
	         * @param y
	         * @return Key with unused sign bit and bit 30
	         */
	        public int makeKey(short x, short y) {
	                short xi = (short) ((x + xoffset) * radix);
	                short yi = (short) ((y + yoffset) * radix);
	                int kxy = 0; 
	                for(int i = 15; i > 0; i--) {
	                	kxy <<= 1;
	                	kxy += (xi >> i) & 1;
	                	kxy <<= 1;
	                	kxy += (yi >> i) & 1;
	                }
	                return kxy;
	    }
	    /**
	     * Generate a point key of max 2,147,483,647 in x and y
	     * @param x
	     * @param y
	     * @return long value with sign and bit 62 unused
	     */
	    public long makeKey(int x, int y) {
                int xi = (int) ((x + xoffset) * radix);
                int yi = (int) ((y + yoffset) * radix);
                long kxy = 0; 
                for(int i = 31; i > 0; i--) {
                	kxy <<= 1;
                	kxy += (xi >> i) & 1;
                	kxy <<= 1;
                	kxy += (yi >> i) & 1;
                }
                return kxy;
        }
	    /**
	     * Generate a square key of 16384 in x1, y1, x2, y2
	     * @param x1
	     * @param y1
	     * @param x2
	     * @param y2
	     * @return long key with first 4 bits unused
	     */
	    public long makeKey(short x1, short y1, short x2, short y2) {
            short xi = (short) ((x1 + xoffset) * radix);
            short yi = (short) ((y1 + yoffset) * radix);
            short xj = (short) ((x2 + xoffset) * radix);
            short yj = (short) ((y2 + yoffset) * radix);
            long kxy = 0; 
            for(int i = 15; i > 0; i--) {
            	kxy <<= 1;
            	kxy += (xi >> i) & 1;
            	kxy <<= 1;
            	kxy += (yi >> i) & 1;
              	kxy <<= 1;
            	kxy += (xj >> i) & 1;
            	kxy <<= 1;
            	kxy += (yj >> i) & 1;
            }
            return kxy;
        }
	    /**
	     * Generate a cube key of 16384 in x1, y1, x2, y2, x3, y3
	     * @param x1
	     * @param y1
	     * @param x2
	     * @param y2
	     * @param x3
	     * @param y3
	     * @return
	     */
	    public BigInteger makeKey(short x1, short y1, short x2, short y2, short x3, short y3) {
            short xi = (short) ((x1 + xoffset) * radix);
            short yi = (short) ((y1 + yoffset) * radix);
            short xj = (short) ((x2 + xoffset) * radix);
            short yj = (short) ((y2 + yoffset) * radix);
            short xk = (short) ((x3 + xoffset) * radix);
            short yk = (short) ((y3 + yoffset) * radix);
            BigInteger kxy = BigInteger.ZERO; 
            for(int i = 15; i >= 0; i--) {
            	kxy = kxy.shiftLeft(1);
            	kxy = kxy.add(BigInteger.valueOf((xi >> i) & 1));
            	kxy = kxy.shiftLeft(1);
            	kxy = kxy.add(BigInteger.valueOf((yi >> i) & 1));
            	kxy = kxy.shiftLeft(1);
              	kxy = kxy.add(BigInteger.valueOf((xj >> i) & 1));
              	kxy = kxy.shiftLeft(1);
            	kxy = kxy.add(BigInteger.valueOf((yj >> i) & 1));
            	kxy = kxy.shiftLeft(1);
              	kxy = kxy.add(BigInteger.valueOf((xk >> i) & 1));
              	kxy = kxy.shiftLeft(1);
            	kxy = kxy.add(BigInteger.valueOf((yk >> i) & 1));
            }
            return kxy;
        }
	    // single argument puts applicable when key has already been formed, or deformed and reformed
		public V put(Integer skey, V value) {
		       return treeMap.put((T)convertInstanceOfObject(skey), value);
		}
		public V put(Long skey, V value) {
		       return treeMap.put((T)convertInstanceOfObject(skey), value);
		}
		public V put(BigInteger skey, V value) {
		       return treeMap.put((T)convertInstanceOfObject(skey), value);
		}
		
		// Multiple args make key then insert
	    public V put(short x, short y, V tvalue) {       
	       int trtn = makeKey(x, y);
	       return treeMap.put((T)convertInstanceOfObject(trtn), tvalue);
	    }
	    public V put(short x, short y, int options, V tvalue) {       
		       int trtn = makeKey(x, y);
		       long ltrtn = (trtn << 32) | options;
		       return treeMap.put((T)convertInstanceOfObject(ltrtn), tvalue);
		}
		public V put(int i, int j, V tvalue) {
			long trtn = makeKey(i, j);
			return treeMap.put((T)convertInstanceOfObject(trtn), tvalue);
		}
	    public V put(short x1, short y1, short x2, short y2, V tvalue) {       
		       long trtn = makeKey(x1, y1, x2, y2);
		       return treeMap.put((T)convertInstanceOfObject(trtn), tvalue);
		}
	    public V put(short x1, short y1, short x2, short y2, short x3, short y3, V tvalue) {       
		       BigInteger trtn = makeKey(x1, y1, x2, y2, x3, y3);
		       return treeMap.put((T)convertInstanceOfObject(trtn), tvalue);
		}
	    public static <T> T convertInstanceOfObject(Object o) {
	        try {
	            return (T)o;
	        } catch(ClassCastException e) {
	        	System.out.println("Cant cast "+o.getClass()+" "+e);
	            return null;
	        }
	    }
   
	    public TreeMap<T, V> getTreeMap() { return treeMap; }
	    
	    /**
	    * Get range
	    * ranges use a 0xFFFFFFFF or 0xFFFFFFFE 0xFFFFFFFC for lowMask and 0x1 or 0x3 for hiMask for instance
	    */
	    public SortedMap<T,V> subMap(short x, short y, int lowMask, int hiMask) {
	       // flattened bit range, may lose some on bounds
	       // rtn.radixKey &= 0x7FFFFFFFFFF00000L;
	       // rtnHigh.radixKey = rtn.radixKey | 0xFFFFFL;
	       // we make a real range, but it costs a little
	       int rtn = makeKey(x,y);
	       int rtnLow = rtn & lowMask;
	       int rtnHigh = rtn | hiMask;
	       if( rtnLow < 0 || rtnHigh < 0)
	    	   System.out.println("RadixTree SubMap KEY OVERFLOW:"+x+" "+y+" mask:"+lowMask+" "+hiMask+" low key="+rtnLow+" hi key="+rtnHigh);
//	       System.out.println(rtn.radixKey+" "+rtnHigh.radixKey);
	       return treeMap.subMap((T)convertInstanceOfObject(rtnLow), (T)convertInstanceOfObject(rtnHigh));
	    }
	    /**
	     * Get range based on 2 coords.
	     * @param x
	     * @param y
	     * @param keyBitMagnitude Each degree is 2 bits, so this is the number of 2 bit pairs to mask
	     * @return
	     */
	    public SortedMap<T,V> subMap(short x, short y, int keyBitMagnitude) {
	        int lowMask = 0;
	        int hiMask = 0;
			for(int i = 0; i <= keyBitMagnitude * 2; i++) {
				hiMask = (hiMask << 1) | 1;
			}
			for(int i = 32; i > keyBitMagnitude * 2; i--) {
				lowMask = (lowMask >>> 1) | 0x40000000;
			}
		   int rtn = makeKey(x,y);
		   int rtnLow = rtn & lowMask;
		   int rtnHigh = rtn | hiMask;
		   if( rtnLow < 0 || rtnHigh < 0)
		    System.out.println("RadixTree SubMap KEY OVERFLOW:"+x+" "+y+" mask:"+lowMask+" "+hiMask+" low key="+rtnLow+" hi key="+rtnHigh);
//		       System.out.println(rtn.radixKey+" "+rtnHigh.radixKey);
		   return treeMap.subMap((T)convertInstanceOfObject(rtnLow), (T)convertInstanceOfObject(rtnHigh));
		}
	    /**
	    * Get range 
	    * 0xFFFFFFFFFFFFFFFF  or 0xFFFFFFFFFFFFFFFE or 0xFFFFFFFFFFFFFFFC low 0x01 or 0x03 for high
	    */
	    public SortedMap<T,V> subMap(int x, int y, long lowMask, long hiMask) {
	       long rtn = makeKey(x,y);
	       long rtnLow = rtn & lowMask;
	       long rtnHigh = rtn | hiMask;
	       if( rtnLow < 0 || rtnHigh < 0)
	    	   System.out.println("RadixTree SubMap KEY OVERFLOW:"+x+" "+y+" mask:"+lowMask+" "+hiMask+" low key="+rtnLow+" hi key="+rtnHigh);
//	       System.out.println(rtn.radixKey+" "+rtnHigh.radixKey);
	       return treeMap.subMap((T)convertInstanceOfObject(rtn), (T)convertInstanceOfObject(rtnHigh));
	    }
	    /**
	     * Get Range based on 2 int keys.
	     * @param x
	     * @param y
	     * @param keyBitMagnitude each degree is 2 bits, so this is number of 2 bit pairs to mask.
	     * @return
	     */
	    public SortedMap<T,V> subMap(int x, int y, int keyBitMagnitude) {
	        long lowMask = 0;
			long hiMask = 0;
			for(int i = 0; i <= keyBitMagnitude * 2; i++) {
			   hiMask = (hiMask << 1) | 1;
			}
			for(int i = 64; i > keyBitMagnitude * 2; i--) {
			   lowMask = (lowMask >>> 1) | 0x4000000000000000L;
			}
		   long rtn = makeKey(x,y);
		   long rtnLow = rtn & lowMask;
		   long rtnHigh = rtn | hiMask;
		   if( rtnLow < 0 || rtnHigh < 0)
		     System.out.println("RadixTree SubMap KEY OVERFLOW:"+x+" "+y+" mask:"+lowMask+" "+hiMask+" low key="+rtnLow+" hi key="+rtnHigh);
//		       System.out.println(rtn.radixKey+" "+rtnHigh.radixKey);
		   return treeMap.subMap((T)convertInstanceOfObject(rtn), (T)convertInstanceOfObject(rtnHigh));
		}
	    /**
	    * Get range
	    * ranges use a 0xFFFFFFFFFFFFFFFF or 0xFFFFFFFFFFFFFFFE 0xFFFFFFFFFFFFFFFC for lowMask and 0x0000000000000001 or 0x000000000000003 for hiMask for instance
	    */
	    public SortedMap<T,V> subMap(short x1, short y1, short x2, short y2, long lowMask, long hiMask) {
	       // flattened bit range, may lose some on bounds
	       // rtn.radixKey &= 0x7FFFFFFFFFF00000L;
	       // rtnHigh.radixKey = rtn.radixKey | 0xFFFFFL;
	       // we make a real range, but it costs a little
	       long rtn = makeKey(x1,y1,x2,y2);
	       long rtnLow = rtn & lowMask;
	       long rtnHigh = rtn | hiMask;
	       if( rtnLow < 0 || rtnHigh < 0)
	    	   System.out.println("RadixTree SubMap KEY OVERFLOW:"+x1+" "+y1+" "+x2+" "+y2+" mask:"+lowMask+" "+hiMask+" low key="+rtnLow+" hi key="+rtnHigh);
//	       System.out.println(rtn.radixKey+" "+rtnHigh.radixKey);
	       return treeMap.subMap((T)convertInstanceOfObject(rtnLow), (T)convertInstanceOfObject(rtnHigh));
	    }
	    /**
	     * Get range based on 4 coords.
	     * @param x1
	     * @param y1
	     * @param x2
	     * @param y2
	     * @param keyBitMagnitude Each degree is 4 bits, so this is the number of 4 bit pairs to mask.
	     * @return
	     */
	    public SortedMap<T,V> subMap(short x1, short y1, short x2, short y2, int keyBitMagnitude) {
	        long lowMask = 0;
		    long hiMask = 0;
		    for(int i = 0; i <= keyBitMagnitude * 4; i++) {
		    	hiMask = (hiMask << 1) | 1;
		    }
		    for(int i = 64; i > keyBitMagnitude * 4; i--) {
		    	lowMask = (lowMask >>> 1) | 0x4000000000000000L;
		    }
		    long rtn = makeKey(x1,y1,x2,y2);
		    long rtnLow = rtn & lowMask;
		    long rtnHigh = rtn | hiMask;
		    if( rtnLow < 0 || rtnHigh < 0)
		     System.out.println("RadixTree SubMap KEY OVERFLOW:"+x1+" "+y1+" "+x2+" "+y2+" mask:"+lowMask+" "+hiMask+" low key="+rtnLow+" hi key="+rtnHigh);
//		     System.out.println(rtn.radixKey+" "+rtnHigh.radixKey);
		    return treeMap.subMap((T)convertInstanceOfObject(rtnLow), (T)convertInstanceOfObject(rtnHigh));
		 }
	    /**
	    * Get range
	    * ranges use a 0xFFFFFFFFFFFFFFFFFFFFFFFF for lowMask and 0x00000000000000000000001 for hiMask for instance
	    */
	    public SortedMap<T,V> subMap(short x1, short y1, short x2, short y2, short x3, short y3, BigInteger lowMask, BigInteger hiMask) {
	       // flattened bit range, may lose some on bounds
	       // rtn.radixKey &= 0x7FFFFFFFFFF00000L;
	       // rtnHigh.radixKey = rtn.radixKey | 0xFFFFFL;
	       // we make a real range, but it costs a little
	       BigInteger rtn = makeKey(x1,y1,x2,y2,x3,y3);
	       BigInteger rtnLow = rtn.and(lowMask);
	       BigInteger rtnHigh = rtn.or(hiMask);
	       if( rtnLow.compareTo(BigInteger.ZERO) < 0 || rtnHigh.compareTo(BigInteger.ZERO) < 0)
	    	   System.out.println("RadixTree SubMap KEY OVERFLOW:"+x1+" "+y1+" "+x2+" "+y2+" mask:"+lowMask+" "+hiMask+" low key="+rtnLow+" hi key="+rtnHigh);
//	       System.out.println(rtn.radixKey+" "+rtnHigh.radixKey);
	       return treeMap.subMap((T)convertInstanceOfObject(rtnLow), (T)convertInstanceOfObject(rtnHigh));
	    }
	    /**
	     * Get range based on 6 coords.
	     * @param x1
	     * @param y1
	     * @param x2
	     * @param y2
	     * @param x3
	     * @param y3
	     * @param keyBitMagnitude Each degree is 6 bits, so this is the number of 6 bit pairs to mask.
	     * @return
	     */
	    public SortedMap<T,V> subMap(short x1, short y1, short x2, short y2, short x3, short y3, int keyBitMagnitude) {
	       // flattened bit range, may lose some on bounds
	       // we make a real range, but it costs a little
	       BigInteger lowMask = BigInteger.ZERO;
	       BigInteger hiMask = BigInteger.ZERO;
	       for(int i = 0; i <= keyBitMagnitude * 6; i++) {
	    	  hiMask = hiMask.setBit(i);
	       }
	       for(int i = 96; i > keyBitMagnitude * 6; i--) {
	    	  lowMask = lowMask.setBit(i);
	       }
	       BigInteger rtn = makeKey(x1,y1,x2,y2,x3,y3);
	       BigInteger rtnLow = rtn.and(lowMask);
	       BigInteger rtnHigh = rtn.or(hiMask);
	       if(DEBUG) {
	    	   System.out.println("Low mask="+String.format("%096x",lowMask)+" High mask="+String.format("%096x",hiMask)+
	    			   " low key="+String.format("%096x",rtnLow)+" High key="+String.format("%096x",rtnHigh));
	       }
	       if( rtnLow.compareTo(BigInteger.ZERO) < 0 || rtnHigh.compareTo(BigInteger.ZERO) < 0)
	    	   System.out.println("RadixTree SubMap KEY OVERFLOW:"+x1+" "+y1+" "+x2+" "+y2+" mask:"+lowMask+" "+hiMask+" low key="+rtnLow+" hi key="+rtnHigh);
//	       System.out.println(rtn.radixKey+" "+rtnHigh.radixKey);
	       return treeMap.subMap((T)convertInstanceOfObject(rtnLow), (T)convertInstanceOfObject(rtnHigh));
	    }

}
