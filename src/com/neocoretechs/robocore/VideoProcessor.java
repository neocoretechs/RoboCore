package com.neocoretechs.robocore;

import java.awt.Color;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.BrokenBarrierException;
import java.util.concurrent.CyclicBarrier;
import java.util.concurrent.atomic.AtomicInteger;

import javax.imageio.ImageIO;

import org.ros.concurrent.CircularBlockingDeque;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import com.neocoretechs.machinevision.CannyEdgeDetector;
import com.neocoretechs.machinevision.hough2d.HoughElem;
import com.neocoretechs.machinevision.hough2d.HoughTransform;
import com.neocoretechs.machinevision.hough3d.Vector4d;
import com.neocoretechs.machinevision.hough3d.hough_settings;
import com.neocoretechs.machinevision.hough3d.octree_t;
import com.neocoretechs.machinevision.hough3d.writer_file;

/**
 * Create a disparity map for the left and right images taken from stereo cameras published to bus.
 * We break the image into  bands and assign each band to a processing thread. 
 * We have an option to write files that can be read by CloudCompare for testing.
 *  __mode:= option on commandline determines how we process, such as 'edge','display', 'display3D',etc.
 *  
 * The new algorithm for stereo coplanar area matching principal component analysis (SCAMPCA). This fast algorithm for
 * assigning depth to stereo point clouds is independent of the presence of color images, works well on low
 * resolution images, requires no calibration of cameras, and is immune to misalignment of the cameras. 
 * 
 * New algorithm for stereo matching:
 * (The terms region, node, and octree node and octree cell are synonymous)
 * The steps are:
 * 1.) Edge detect both images using Canny, gradient level set high for max detail
 *
 * 2.) Generate 2 octrees of edge detected images at the minimal node level, now 7. this step in parallel by assigning
 * the octree build to one Y image scan line per thread.
 *
 * 3.) Use PCA on the 2 images to find minimal coplanar regions in both octrees and generate eigenvectors 
 * and eigenvalues of the principal axis of the minimal coplanar regions. this step in a single thread after barrier 
 * synchronization of the build step. Multiple threads not necessary as this step is fast.
 *
 * 4.) Process the left image coplanar regions against the right coplanar regions by comparing 
 * the vector cross products of the eigenvectors second and third axis for all minimal right regions in the
 * yTolerance of the left Y scan line assigned to the thread calling the method. 
 * Experiments have found that this yields a single candidate in most real world cases so far but the code is such 
 * that in case of multiple matches the one with the smallest difference in eigenvector values to the left region is chosen.
 * The minimum points per octree cell can be changed in hough_settings but typically minimum 5 points. 
 * The coplanar region found by PCA may contain more however.
 * This is analogous to the standard stereo block matching (SBM) done by brute force sum of absolute differences (SAD) of the RGB
 * values of left and right windows of pixels, but instead using a more evolved approach of comparing the essence 
 * of the orientation of small regions of marker points in each image, more akin to the way organisms seem to do it.
 *
 * 5.) Assign a disparity to the minimal octree region based on differences in epipolar distance values of the 2 octree 
 * centroids of the points in the coplanar minimal regions from step 4. The regions are small enough that 
 * centroid value is accurate enough to give reasonable disparity, identical to standard black matching
 *
 * 6.) Reset left image octree and regenerate coplanar regions via PCA at a higher octree level, using level 5. The concept
 * is that regions that were coplanar on a smaller level and are coplanar at a higher level still, form one region of interest.
 *
 * 7.) Find the minimal regions (that were assigned a depth) enclosed by the newly generated larger coplanar regions.
 *
 * 8.) For all points in the larger regions, determine which enclosed minimal region they fall into. If within the 
 * smaller region, assign the point the depth we found and assigned to that minimal region earlier in step 5.
 *
 * 9.) For points in the larger region that do not fall into a smaller enclosed region, find the closest edge of 
 * a smaller enclosed region they are near and assign the depth of that region to the point.
 *
 * 10.) After processing a larger region, remove the smaller regions it encloses from consideration.
 *
 * 11.) Regenerate coplanar regions via PCA  at a higher octree level, using level 4, if there are unprocessed smaller
 *  regions (regions that were not enclosed by a larger one) .
 *
 * 12.) For all minimal regions not previously processed, perform steps 7,8,9,10 although it seems that in 
 * practice all regions are processed without having to use the larger octree nodes at level 4.
 * 
 * Once completed, a set of coplanar octree regions that can be used to navigate or for further processing into larger coplanar
 * areas is available.
 *
 * @author jg (C) NeoCoreTechs 2018,2019
 *
 */
public class VideoProcessor extends AbstractNodeMain 
{
	private static boolean DEBUG = false;
	private static boolean DEBUGTEST3 = false;
	private static boolean DEBUGTEST2 = false;
	private static boolean DEBUGTEST4 = false;
	private static final boolean SAMPLERATE = false; // display pubs per second
	private static final boolean TIMER = true;
	private static final boolean WRITEFILES = false;
	private static final boolean ASSIGNPOINTS = false;

    private BufferedImage imageL = null;
    private BufferedImage imageR = null;
    private BufferedImage imageLx = null;
    private BufferedImage imageRx = null;

	//int outWidth = 1280;
	//int outHeight = 1000;
	int outWidth = 640;
	int outHeight = 480;
    
    ByteBuffer cbL, cbR;
    byte[] bufferL = new byte[0];
    byte[] bufferR = new byte[0];
   
	String mode = "";
	String outDir = "/users/jg/workspace/robocore/motionclouds";
	int frames = 0;
	int files = 0;
	// optical parameters
    final static float f = 4.4f; // focal length mm
    final static float B = 205.0f; // baseline mm
    final static float FOVD = 60; // degrees field of view
    final static double FOV = 1.04719755; // radians FOV
    final static int camWidth = 640; // pixels
    final static int camHeight = 480;
    final static double fp = B*(camWidth*.5)/Math.tan(FOV *.5 * (Math.PI/2)); //focal length in pixels
    final static double Bf = B*f;// calc the Bf of Bf/d
    final static int maxHorizontalSep = (int) (Bf-1)/4; // max pixel disparity
    final static int yTolerance = 25; // pixel diff in y of potential candidates

    CircularBlockingDeque<BufferedImage> queueL = new CircularBlockingDeque<BufferedImage>(10);
    CircularBlockingDeque<BufferedImage> queueR = new CircularBlockingDeque<BufferedImage>(10);
	
	private int sequenceNumber,lastSequenceNumber;
	long time1;

	//FloatDCT_2D fdct2dL = new FloatDCT_2D(corrWinSize, corrWinSize);
	//FloatDCT_2D fdct2dR = new FloatDCT_2D(corrWinSize, corrWinSize);

    CannyEdgeDetector ced = null;
    
	BufferedImage bimage = null;
	octree_t node = null;
	octree_t nodel = null;
	octree_t noder = null;
	List<envInterface> indexDepth; // correlated and matched minimal regions
	List<envInterface> indexUnproc; // uncorrelated minimal regions
	List<envInterface> maxEnv; // maximal regions that enclose one or more minimal regions
	List<envInterface> zeroEnc; // maximal regions that enclose zero minimal regions
	List<int[]> leftYRange; // from/to position in array for sorted Y centroid processing
	int[] dataL; // left array return from canny with magnitudes
	int[] dataR; // right canny array with magnitudes
	//CyclicBarrier latch = new CyclicBarrier(camHeight/corrWinSize+1);
	//CyclicBarrier latchOut = new CyclicBarrier(camHeight/corrWinSize+1);
	CyclicBarrier latch = new CyclicBarrier(2);
	CyclicBarrier latchOut = new CyclicBarrier(2);
	CyclicBarrier latch2 = new CyclicBarrier(2);
	CyclicBarrier latchOut2 = new CyclicBarrier(2);
	CyclicBarrier latch3 = new CyclicBarrier(2);
	CyclicBarrier latchOut3 = new CyclicBarrier(2);
	CyclicBarrier latch4 = new CyclicBarrier(2);
	CyclicBarrier latchOut4 = new CyclicBarrier(2);
	//int yStart;
	int threads = 0;
	double mean, sigma, variance;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("subs_videoproc");
	}
	
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		//Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
		// bring up all parallel  processing threads
		spinUp();
		// subscribe to stereo image bus to start flow of images into queue
		final Subscriber<stereo_msgs.StereoImage> imgsub =
				connectedNode.newSubscriber("/stereo_msgs/StereoImage", stereo_msgs.StereoImage._TYPE);
		/**
		 * Image extraction from bus, then image processing, then on to display section.
		 */
		imgsub.addMessageListener(new MessageListener<stereo_msgs.StereoImage>() {
		@Override
		public void onNewMessage(stereo_msgs.StereoImage img) {
			long slew = System.currentTimeMillis() - time1;
			if( SAMPLERATE && slew >= 1000) {
				time1 = System.currentTimeMillis();
				System.out.println("Samples per second:"+(sequenceNumber-lastSequenceNumber)+". Slew rate="+(slew-1000));
				lastSequenceNumber = sequenceNumber;
			}
			try {
				//synchronized(mutex) {
					cbL = img.getData();
					bufferL = cbL.array();
					InputStream in = new ByteArrayInputStream(bufferL);
					BufferedImage imageL1 = ImageIO.read(in);
					in.close();
					cbR = img.getData2();
					bufferR = cbR.array();
					in = new ByteArrayInputStream(bufferR);
					BufferedImage imageR1 = ImageIO.read(in);
					in.close();
					queueL.addLast(imageL1);
					queueR.addLast(imageR1);
				//}
				//IntBuffer ib = cb.toByteBuffer().asIntBuffer();
				if( DEBUG ) {
					System.out.println("New left/right images "+img.getWidth()+","+img.getHeight()+" size:"+bufferL.length/*ib.limit()*/);
				}
			} catch (IOException e1) {
				System.out.println("Could not convert image payload due to:"+e1.getMessage());
				return;
			}
			++sequenceNumber; // we want to inc seq regardless to see how many we drop	
		}
	  });
		
	} // onstart
	/**
	 * Spin all parallel processing threads
	 */
	public void spinUp() {
		/*
		 * Main worker thread for image data. 
		 * Wait at synch barrier for completion of all processing threads, then display disparity 
		 */
		final AtomicInteger yStart = new AtomicInteger(0);
		ThreadPoolManager.getInstance().spin(new Runnable() {
				@Override
				public void run() {
					System.out.println("Processing "+camWidth+" by "+camHeight);
					while(true) {
					//
					// Spin the threads for each chunk of image
					// Since we run a continuous loop inside run we have to have a way
					// to synchronize everything at the beginning of a new image, then
					// await completion of all processing threads and signal a reset to
					// resume processing at the start of another new image, hence the two barrier
					// synchronization latches
					//
					try {
					  latch.await();
					  long etime = System.currentTimeMillis();
					  yStart.set(0);
					  int numThreads = camHeight/10;
					  int execLimit = camHeight;
					  //
					  // spin all threads necessary for execution
					  //
					  for(int syStart = 0; syStart < execLimit; syStart++) {
						SynchronizedFixedThreadPoolManager.getInstance(numThreads, execLimit).spin(new Runnable() {
						  @Override
						  public void run() {
							// Since we run a continuous loop inside run we have to have a way
							// to synchronize everything at the beginning of a new image, then
							// await completion of all processing threads and signal a reset to
							// resume processing at the start of another new image, hence the two barrier
							// synchronization latches
								imagesToOctrees(dataL, dataR, imageLx, imageRx, yStart.getAndIncrement(), camWidth, camHeight, nodel, noder/*hlL, hlR*/);
						  } // run
					    }); // spin
					  } // for syStart
					  SynchronizedFixedThreadPoolManager.getInstance().waitForGroupToFinish();
					  if( TIMER )
						  System.out.println("Process time one="+(System.currentTimeMillis()-etime));
					  latchOut.await();
					  //
					  // next parallel processing step, if any
					  //
					  final Comparator<octree_t> yComp = new Comparator<octree_t>() {         
						  @Override         
						  public int compare(octree_t jc1, octree_t jc2) {             
							      return (jc2.getCentroid().y < jc1.getCentroid().y ? -1 :                     
							              (jc2.getCentroid().y == jc1.getCentroid().y ? 0 : 1));           
						  }     
					  }; 
					  latch2.await();
					  etime = System.currentTimeMillis();
					  yStart.set(0);
					  ArrayList<octree_t> tnodel = nodel.get_nodes();
					  ArrayList<octree_t> tnoder = noder.get_nodes();
					  Collections.sort(tnodel, yComp);
					  Collections.sort(tnoder, yComp);
					  final List<octree_t> nodelA = Collections.synchronizedList(tnodel);
					  final List<octree_t> noderA = Collections.synchronizedList(tnoder);
					  indexDepth = Collections.synchronizedList(new ArrayList<envInterface>());
					  indexUnproc = Collections.synchronizedList(new ArrayList<envInterface>());
					  leftYRange = Collections.synchronizedList(new ArrayList<int[]>());
					  // form list of start/end common Y values from sorted Y centroid list to partition parallel work
					  double y = nodelA.get(0).getMiddle().y;
					  int iPosStart = 0;
					  int iPosEnd = 0;
					  for(int i = 0 ; i < nodelA.size(); i++) {
						   if( y != nodelA.get(i).getCentroid().y) {
							   iPosEnd = i;
							   leftYRange.add(new int[]{iPosStart, iPosEnd});
							   iPosStart = i;
							   y = nodelA.get(i).getCentroid().y;
						   }
					  }
					  iPosEnd = nodelA.size();
					  leftYRange.add(new int[]{iPosStart, iPosEnd});
					  final int nSize = leftYRange.size();
					  final int nSizeT = Math.min(leftYRange.size(), 32);
					  SynchronizedFixedThreadPoolManager.getInstance().init(nSizeT, nSize, "MATCHREGION");
					  for(int syStart = 0; syStart < nSize; syStart++) {
							SynchronizedFixedThreadPoolManager.getInstance(nSizeT, nSize, "MATCHREGION").spin(new Runnable() {
								@Override
								public void run() {
									// set the left nodes with depth
									matchRegionsAssignDepth(yStart.getAndIncrement(), leftYRange, camWidth, camHeight, nodelA, noderA, indexDepth, indexUnproc);
								} // run									
							},"MATCHREGION"); // spin
					  } // for syStart
					  SynchronizedFixedThreadPoolManager.getInstance().waitForGroupToFinish("MATCHREGION");
					  if( TIMER )
							System.out.println("Process time two="+(System.currentTimeMillis()-etime));
					  latchOut2.await();
					  //
					  // next parallel processing step, if any
					  //
					  latch3.await();
					  etime = System.currentTimeMillis();
					  yStart.set(0);
					  maxEnv = Collections.synchronizedList(new ArrayList<envInterface>());
					  zeroEnc = Collections.synchronizedList(new ArrayList<envInterface>());
					  // Octree reset to higher level, re-get nodes
					  final List<octree_t> nodelB = Collections.synchronizedList(nodel.get_nodes());
					  final int nSizeu = nodelB.size();
					  final int nSizeU = Math.min(nodelB.size(), 32);
					  // gen 1 thread for each array element up to limit
					  SynchronizedFixedThreadPoolManager.getInstance().init(nSizeU, nSizeu, "SETPOINT");
					  for(int syStart = 0; syStart < nSizeu; syStart++) {
						SynchronizedFixedThreadPoolManager.getInstance(nSizeU, nSizeu, "SETPOINT").spin(new Runnable() {
							@Override
							public void run() {
								// set the left nodes with depth
								findEnclosedRegionsSetPointDepth(yStart.getAndIncrement(), nodelB, indexDepth, maxEnv, zeroEnc);
							} // run									
						},"SETPOINT"); // spin
					  } // for syStart
					  SynchronizedFixedThreadPoolManager.getInstance().waitForGroupToFinish("SETPOINT");
					  if( TIMER )
							System.out.println("Process time three="+(System.currentTimeMillis()-etime));
					  latchOut3.await();
					  //
					  // next parallel processing step, if any
					  //
					  if( !zeroEnc.isEmpty() ) {
						latch4.await();
						etime = System.currentTimeMillis();
						yStart.set(0);
						final int nSizex = zeroEnc.size();
						final int nSizeX = Math.min(zeroEnc.size(), 32);
						// gen 1 thread for each array element
						SynchronizedFixedThreadPoolManager.getInstance().init(nSizeX, nSizex, "ZEROPOINT");
						for(int syStart = 0; syStart < nSizex; syStart++) {
								SynchronizedFixedThreadPoolManager.getInstance(nSizeX, nSizex, "ZEROPOINT").spin(new Runnable() {
									@Override
									public void run() {
										// set the left nodes with depth
										findZeroEnclosedSetPointDepth(yStart.getAndIncrement(), maxEnv, zeroEnc);
									} // run									
								},"ZEROPOINT"); // spin
						} // for syStart
						SynchronizedFixedThreadPoolManager.getInstance().waitForGroupToFinish("ZEROPOINT");
						if( TIMER )
							System.out.println("Process time four="+(System.currentTimeMillis()-etime));
						latchOut4.await();
					  }
					  
					// global barrier break
					} catch (BrokenBarrierException | InterruptedException e) { System.out.println("<<BARRIER BREAK>> "+this);}
					} // while true
			} // run
		}); // spin
					
		/**
		 * Main processing thread for image data. Extract image queue elements from ROS bus and then
		 * notify waiting worker threads to process them.
		 * Wait at synch barrier for completion of all processing threads, then display disparity 
		 */
		ThreadPoolManager.getInstance().spin(new Runnable() {			
				public void run() {
					System.out.println("Image queue..");
					/**
					 * Main processing loop, extract images from queue, notify worker threads, then display disparity
					 */
			        while(true) {
			        	if( queueL.isEmpty() || queueR.isEmpty() ) {
							try {
								Thread.sleep(1);
							} catch (InterruptedException e) {}
			        	} else {
			        		try {
			        			// If we are waiting at either cyclic barrier, the reset will cause it to
			        			// to return to top level barrier
			        			// the latches represent each 'step', or group of parallel processing tasks
			        			// use as many steps as needed, unused latches ignored
								imageLx = queueL.takeFirst();
				        		imageRx = queueR.takeFirst();
			        			latchOut4.reset();
			        			latch4.reset();
			        			latchOut3.reset();
			        			latch3.reset();
			        			latchOut2.reset();
			        			latch2.reset();
			        			latchOut.reset();
			        			latch.reset();
				        		//
				        	    synchronized(imageLx) {
				        	     ced = new CannyEdgeDetector();
				        	     ced.setLowThreshold(0.5f);
				        	     ced.setHighThreshold(1f);
				        	     ced.setSourceImage(imageLx);
				        	     dataL = ced.semiProcess();
				        	     /*
				        			for(int j = 0; j < camWidth; j++) {
				        			for(int i = 0; i < camHeight; i++) {
				        				if( dataL[j*camWidth+i] != 0) {
				        					htL.addPoint(j, i);
				        				}
				        			}
				        			}
				        	      */
				        	     //ced.process();
				        	     //imageL = ced.getEdgesImage();
				        	    }
				        	    synchronized(imageRx) {
				        	     ced = new CannyEdgeDetector();
				        	     ced.setLowThreshold(0.5f);
				        	     ced.setHighThreshold(1f);
				        	     ced.setSourceImage(imageRx);
				        	     dataR = ced.semiProcess();
				        	     /*
				        			for(int j = 0; j < camWidth; j++) {
				        			for(int i = 0; i < camHeight; i++) {
				        				if( dataR[j*camWidth+i] != 0) {
				        					htR.addPoint(j, i);
				        				}
				        			}
				        			}
				        	      */
				        	     //ced.process();
				        	     //imageR = ced.getEdgesImage();
				        	   	}
				        	   	/*
				        	    hlL = htL.getLines(10);
				        	    hlR = htR.getLines(10);
				        	    */
				        	    imageL = imageLx;
				        	    imageR = imageRx;	
			        			nodel = new octree_t();
			        			octree_t.buildStart(nodel);
			        			noder = new octree_t();
			        			octree_t.buildStart(noder);
			        		   	// write source images
			        			if( WRITEFILES ) {
			        				synchronized(imageL) {
			        					writeFile("sourceL", imageL, ++files);
			        				}
			        				synchronized(imageR) {
			        					writeFile("sourceR", imageR, files);
			        				}
			        			}
			        	     	// first step end multi thread barrier synch
			        			latch.await();
			        	     	latchOut.await();
			        	     	synchronized(nodel) {
			        	     		octree_t.buildEnd(nodel);
			        	     		hough_settings.s_level = 7;
			        	     		hough_settings.max_distance2plane = 5;
			        	     		hough_settings.min_isotropy = 0; // allow all sorts of deformation for now
			        	     		nodel.subdivide();
			        	     		//writeFile(nodel,"/roscoeL"+(++frames));
			        	     		if( WRITEFILES)
			        	     			writer_file.writePerp(nodel, "planarsL"+files);
			        	     	}
			        	     	synchronized(noder) {
			        	     		octree_t.buildEnd(noder);
			        	     		noder.subdivide();
			        	     		if( WRITEFILES) {
			        	     			writeFile(noder,"/roscoeR"+files);
			        	     			writer_file.writePerp(noder, "planarsR"+files);
			        	     		}
			        	     	}
			        	     	// second step end multi thread barrier synch
			        	     	latch2.await();
			        	     	latchOut2.await();
			        	     	// write unmatched minimal envelopes
			        	     	if( WRITEFILES) {
			        	     		synchronized(indexUnproc) {
			        	     			writeFile(indexUnproc, "/lvl7uncorrL"+files);
			        	     		}
			        	     	}
								// calc mean
								mean = variance = 0;
								synchronized(indexDepth) {
									  for(envInterface ind : indexDepth) {
										  mean += ind.getDepth();
									  }
									  mean /= (double)indexDepth.size();
									  for(envInterface ind : indexDepth) {
										  variance += Math.pow((mean - ind.getDepth()),2);
									  }
									  variance /= (double)indexDepth.size();
								}
								sigma = Math.sqrt(variance);
								System.out.println("Mean="+mean+" variance="+variance+" standard deviation="+sigma);
			        	     	System.out.println("Uncorrelated regions="+indexUnproc.size()+" correlated="+indexDepth.size()+", "+(((float)indexUnproc.size()/(float)(indexUnproc.size()+indexDepth.size()))*100.0)+"%");
			        	     	// reset level then regenerate tree with maximal coplanar points
			        	     	synchronized(nodel) {
			        	     		// set our new maximal level
			        	     		hough_settings.s_level = 5;
			        	     		// set the distance to plane large to not throw out as outliers points we recently assigned a z
			        	     		// this is a divisor so we set to 1
			        	     		hough_settings.max_distance2plane = 1;
			        	     		hough_settings.min_isotropy = .01; // prevent elongated or vertical funky planes
			        	     		nodel.clear();
			        	     		nodel.subdivide();
			        	     		// write the display cloud with maximal envelopes
			        	     		//if(WRITEFILES)
			        	     		//		writer_file.writeEnv(nodel, "lvl4planenvL"+files);
			        	     	}
			        	     	// third step wait for completion
			        	     	latch3.await();
			        	     	latchOut3.await();
			        	     	// start step 4
			        	     	System.out.println("Unenclosed minimal envelopes="+indexDepth.size()+" zero enclosing maximal envelopes="+zeroEnc.size());
			        	     	
		        	     		// if any unenclosing envelopes, try to assign points using another enclosed maximal area
			        	     	if( !zeroEnc.isEmpty()) {
			        	     		latch4.await();
			        	     		latchOut4.await();
			        	     		int iZer = 0;
			        	     		for(envInterface e : zeroEnc) {
			        	     			if(e.getDepth() == 0)
			        	     				++iZer;
			        	     		}
			        	     		// write the display cloud with maximal envelopes
			        	     		if(WRITEFILES)
			        	     				writeFile(zeroEnc, "/lvl5zeroenvL"+files);
			        	     		System.out.println("Final zero enclosing maximal envelopes="+iZer+" out of "+zeroEnc.size());
			        	     	}
			        	     	
			        	     	// end of parallel processing
			        	     	synchronized(nodel) {
			        	     			// set our new maximal level
			        	     			hough_settings.s_level = 5;
			        	     			// set the distance to plane to increase order in plane formation
			        	     			// this is a divisor
			        	     			hough_settings.max_distance2plane = 5;
				        	     		hough_settings.min_isotropy = .015; // increase order among planes
			        	     			nodel.clear();
			        	     			nodel.subdivide();
			        	     			// write the display cloud with maximal planes detected
			        	     			if(WRITEFILES) {
			        	     				writer_file.writePerp(nodel, "planesL"+files);
			        	     			}
			        	     	}
			        	     	if(WRITEFILES) {
			        	     		// write the remaining unprocessed envelopes from minimal
			        	     		if(!indexDepth.isEmpty())
			        	     			synchronized(indexDepth) {
			        	     				writeFile(indexDepth,"/lvl7unencL"+files);
			        	     			}
			        	     		// at this point the entire point set should be loaded with z
			        	     		synchronized(nodel) {
			        	     			writeFile(nodel,"/roscoeL"+files);
			        	     		}
			        	     	}
							} catch (InterruptedException | BrokenBarrierException e) {
								e.printStackTrace();
							}
			        	}
			        } // while true
				} // run      
		}); // spin
	}
	/**
	 * Write CloudCompare point cloud viewer compatible file
	 * for each point - X,Y,Z,R,G,B ascii delimited by space
	 * @param simage2 the processed array chunks of [x][y][0]=R, [x][y][1]=G, [x][y][2]=B, [x][y][3]=D
	 */
	protected void writeFile(double[][][] simage2) {
		DataOutputStream dos = null;
		File f = new File(outDir+"/roscoe"+(++frames)+".asc");
		int ks, ms;
		double os;
		try {
			dos = new DataOutputStream(new FileOutputStream(f));
			for(int y = 0; y < outHeight; y++) {
				for(int x = 0; x < outWidth; x++) {
					// output only valid data
					if( mode.equals("edge") && simage2[x][y][0] == 0 )
						continue;
					// transform to 3D plane offsets
					ks = x - (camWidth/2);
					ms = y - (camHeight/2);
					os = (Bf/2) - simage2[x][y][3] ;//simage2[x][y][3] - (Bf/2);
					dos.writeBytes(String.valueOf((double)ks/*x*/)); // X
					dos.writeByte(' ');
					dos.writeBytes(String.valueOf((double)ms/*y*/));// Y
					dos.writeByte(' ');
					dos.writeBytes(String.valueOf(os/*simage2[x][y][3]*/)); // Z
					dos.writeByte(' ');
					dos.writeBytes(String.valueOf((int)simage2[x][y][0])); // R
					dos.writeByte(' ');
					dos.writeBytes(String.valueOf((int)simage2[x][y][1])); // G
					dos.writeByte(' ');
					dos.writeBytes(String.valueOf((int)simage2[x][y][2])); // B
					dos.writeByte('\r');
					dos.writeByte('\n');
				}
		}
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			return;
		} catch (IOException e) {
			e.printStackTrace();
		} finally {
			try {
				if( dos != null ) {
					dos.flush();
					dos.close();
				}
			} catch (IOException e) {
			}		
		}
		
	}
	
	protected void writeFile(octree_t node, String filename) {
		DataOutputStream dos = null;
		File f = new File(outDir+filename+".asc");
		try {
			dos = new DataOutputStream(new FileOutputStream(f));
			for(int i = 0; i < node.m_points.size(); i++) {
				Vector4d pnode = node.m_points.get(i);
				Vector4d pcolor = node.m_colors.get(i);
				dos.writeBytes(String.valueOf(pnode.x)); // X
					dos.writeByte(' ');
					dos.writeBytes(String.valueOf(pnode.y));// Y
					dos.writeByte(' ');
					dos.writeBytes(String.valueOf(pnode.z)); // Z
					dos.writeByte(' ');
					dos.writeBytes(String.valueOf((int)pcolor.x)); // R
					dos.writeByte(' ');
					dos.writeBytes(String.valueOf((int)pcolor.y)); // G
					dos.writeByte(' ');
					dos.writeBytes(String.valueOf((int)pcolor.z)); // B
					dos.writeByte('\r');
					dos.writeByte('\n');
			}
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			return;
		} catch (IOException e) {
			e.printStackTrace();
		} finally {
			try {
				if( dos != null ) {
					dos.flush();
					dos.close();
				}
			} catch (IOException e) {
			}		
		}
		
	}
	protected void writeFile(List<envInterface> d, String filename) {
		DataOutputStream dos = null;
		File f = new File(outDir+filename+".asc");
		try {
			dos = new DataOutputStream(new FileOutputStream(f));
			for(envInterface id : d) {
				writer_file.line3D(dos, (int)id.getEnv()[0]/*xmin*/, (int)id.getEnv()[1]/*ymin*/, 1, 
						(int)id.getEnv()[2]/*xmax*/, (int)id.getEnv()[1]/*ymin*/, 1, 0, 255, 255);
				writer_file.line3D(dos, (int)id.getEnv()[2]/*xmax*/, (int)id.getEnv()[1]/*ymin*/, 1, 
						(int)id.getEnv()[2]/*xmax*/, (int)id.getEnv()[3]/*ymax*/, 1, 0, 255, 255);
				writer_file.line3D(dos, (int)id.getEnv()[2]/*xmax*/, (int)id.getEnv()[3]/*ymax*/, 1, 
						(int)id.getEnv()[0]/*xmin*/, (int)id.getEnv()[3]/*ymax*/, 1, 0, 255, 255);
				writer_file.line3D(dos, (int)id.getEnv()[0]/*xmin*/, (int)id.getEnv()[3]/*ymax*/, 1, 
						(int)id.getEnv()[0]/*xmin*/, (int)id.getEnv()[1]/*ymin*/, 1, 0, 255, 255);
			}
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			return;
		} catch (IOException e) {
			e.printStackTrace();
		} finally {
			try {
				if( dos != null ) {
					dos.flush();
					dos.close();
				}
			} catch (IOException e) {
			}		
		}
		
	}
	protected void writeOctree(double[][][] simage2) {
		octree_t node = new octree_t();
		octree_t.buildStart(node);
		for(int y = 0; y < outHeight; y++) {
			for(int x = 0; x < outWidth; x++) {
				// output only valid data
				if( mode.equals("edge") && simage2[x][y][0] == 0 )
					continue;
				// transform to 3D plane offsets
				double ks = x - (camWidth/2);
				double ms = y - (camHeight/2);
				double os = (Bf/2) - simage2[x][y][3] ;//simage2[x][y][3] - (Bf/2);
				octree_t.build(node, (double)ks, (double)ms, os, simage2[x][y][0], simage2[x][y][1], simage2[x][y][2]);
			}
		}
		octree_t.buildEnd(node);
		node.subdivide();
		writer_file.writePerp(node, "planars"+frames);
	}
	

	/**
	 * Write JPEG file 
	 * @param fname File name
	 * @param image BufferedImage with data
	 * @param findex Sequence to append to file name for consecutive frames
	 */
	protected void writeFile(String fname, BufferedImage image, int findex) {
		File f = new File(outDir+"/"+fname+(findex)+".jpg");
		try {
			ImageIO.write(image, "jpg", f);
			if( DEBUG )
				System.out.println("Wrote: "+fname+findex+".jpg");
		} catch (IOException e) {
			System.out.println("Cant write image "+fname+findex+".jpg");
			return;
		}
	}
	
	
	/**
	 * Translate 2 edge detected image integer linear arrays of edge data and their corresponding RGB images
	 * into 2 octrees where the Z is set to 1. Intended to be 1 scan line in multithreaded parallel thread group.
	 * @param imageL Left image result of Canny edge detector
	 * @param imageR Right image Canny array
	 * @param imageLx2 RGB image source left
	 * @param imageRx2 RGB image source right
	 * @param yStart The Y scan line to process, this should be Atomic Integer incrementing per thread assignment
	 * @param width width of images
	 * @param height height of images
	 * @param nodel Left octree root node, filled by this method partially
	 * @param noder Right octree root node, also built and written to by this method
	 */
	public final void imagesToOctrees(int[] imageL, int[] imageR, 
											BufferedImage imageLx2, BufferedImage imageRx2, 
											int yStart, 
											int width, int height,
											octree_t nodel, octree_t noder) {
		long etime = System.currentTimeMillis();
		int[] imgsrcL = new int[width]; // left image scan line 
 		int[] imgsrcR = new int[width]; // right image scan line
   		int[] imgsrcLx = new int[width]; // left image scan line 
 		int[] imgsrcRx = new int[width]; // right image scan line
 		synchronized(imageL) {
 			// gradient magnitude
 			System.arraycopy(Arrays.copyOfRange(imageL, yStart*width, (yStart+1)*width), 0, imgsrcL, 0, width);
 		}
		synchronized(imageLx2) {
			imageLx2.getRGB(0, yStart, width, 1, imgsrcLx, 0, width);
		}
		synchronized(imageR) {
 			// gradient magnitude
 			System.arraycopy(Arrays.copyOfRange(imageR, yStart*width, (yStart+1)*width), 0, imgsrcR, 0, width);
 		}
		synchronized(imageRx2) {
			imageRx2.getRGB(0, yStart, width, 1, imgsrcRx, 0, width);
		}
	  	for(int xsrc = 0; xsrc < width; xsrc++) {
	  		// If the left image pixel which is the target to receive the depth value is not edge, continue
			if(imgsrcL[xsrc] == 0)
					continue;
			double ks = xsrc - (width/2);
			double ms = yStart - (height/2);
			double os = 1;//(Bf/2) - imgsrcL[xsrc]; // gradient intensity

			synchronized(nodel) {
				octree_t.build(nodel, (double)ks, (double)ms, os, 
					((imgsrcLx[xsrc] & 0x00FF0000) >> 16 ), 
					((imgsrcLx[xsrc] & 0x0000FF00) >> 8), 
					((imgsrcLx[xsrc] & 0x000000FF) ));
			}
		} //++xsrc  move to next x subject in left image at this scanline y
  		for(int xsrc = 0; xsrc < width; xsrc++) {
			// If the left image pixel which is the target to receive the depth value is not edge, continue
			if(imgsrcR[xsrc] == 0)
				continue;
			double ks = xsrc - (width/2);
			double ms = yStart - (height/2);
			double os = 1;//(Bf/2) - imgsrcR[xsrc]; // gradient intensity

			synchronized(noder) {
				octree_t.build(noder, (double)ks, (double)ms, os, 
					((imgsrcRx[xsrc] & 0x00FF0000) >> 16 ), 
					((imgsrcRx[xsrc] & 0x0000FF00) >> 8), 
					((imgsrcRx[xsrc] & 0x000000FF) ));
			}
		} //++xsrc  move to next x subject in left image at this scanline y
			//System.out.println("Y scan="+y);
		//} // next y
		if( SAMPLERATE )
			System.out.println("imagesToOctrees IMAGE SCAN LINE Y="+yStart+" ***** "+Thread.currentThread().getName()+" *** "+(System.currentTimeMillis()-etime)+" ms.***");
	}
	/**
	 * Match the regions at minimal level from left to right image. Weed out the ones beyond yTolerance
	 * and match the arccosine of vector dot product of the secondary eigenvector if the difference between the left
	 * node eigenvectors and right is below tolerance. From the candidates below tolerance, select the one
	 * that has the mnimum difference in eigenvector dot. Take the distance between centroids as disparity.
	 * @param yStart position leftYRange containing from/to positions in left node list for this thread.
	 * @param leftYRange2 list containing from/to of common Y centroid elements in sorted left node list
	 * @param camwidth width of image
	 * @param camheight height of image
	 * @param nodelA list of left octree nodes at smallest level, sorted by centroid y
	 * @param noderA list of right octree nodes at smallest level, sorted by centroid y
	 * @param indexDepth2 resulting array filled with envInterface instances of matched regions
	 * @param indexunproc2 resulting array of envInterface filled with unmatched minimal regions
	 */
	private void matchRegionsAssignDepth(int yStart, 
										List<int[]> leftYRange2, 
										int camwidth, int camheight, 
										List<octree_t> nodelA, List<octree_t> noderA, 
										List<envInterface> indexDepth2, List<envInterface> indexUnproc2) {
		long etime = System.currentTimeMillis();
		//octree_t tnodel = new octree_t();
		//tnodel.getCentroid().y = (double)(yStart-(camheight/2));
		List<octree_t> leftNodes;
		// get all nodes along this Y axis, if any
		// we are just trying to establish a range to sublist
		boolean found = false;
		synchronized(nodelA) {
			/*int i1 = 0;
			int i0 = 0;
			for( octree_t inode : nodelA) {
				if( ((int)inode.getCentroid().y) == (yStart-(camheight/2)) ) {
					if( !found ) { 
						found = true;
						i0 = i1; // found first =, set lower range
					} 
					++i1; // found or not, bump 'to' range	
				} else {
					if( !found ) { // not found, not = , keep searching
						++i1;
					} else {
						break; // was found, now not =
					}
				}
			}
			if(!found) {
				if(DEBUGTEST2)
					System.out.println("matchRegionsAssignDepth no left image nodes at centroid Y="+(yStart-(camheight/2))+" ***** "+Thread.currentThread().getName()+" ***" );
				return;
			}
			leftNodes = nodelA.subList(i0, i1);
			*/
			int[] irange = leftYRange2.get(yStart);
			leftNodes = nodelA.subList(irange[0], irange[1]);
		}
		
		// get the right nodes, where the octree cell has same basic number points and plane nornal
		//if( leftNodes.size() == 0) {
		//	if(DEBUGTEST2)
		//		System.out.println("matchRegionsAssignDepth no left image nodes at centroid Y="+(yStart-(camheight/2))+" ***** "+Thread.currentThread().getName()+" ***" );
		//	return;
		//}
		if(DEBUGTEST2)
			System.out.println("matchRegionsAssignDepth "+leftNodes.size()+" left nodes with centroid Y="+(yStart-(camheight/2))+" ***** "+Thread.currentThread().getName()+" ***");
		// cross product of normals zero and same number points for each left node compared to all right nodes
		//synchronized(noderA) {
		//	for( int i = 0; i < leftNodes.size(); i++) {
		//		for(int j = 0; j < noderA.size(); j++) {
		//			if( leftNodes.get(i).getIndexes().size() == noderA.get(j).getIndexes().size() &&
		//				leftNodes.get(i).getNormal1().multiplyVectorial(noderA.get(j).getNormal1()).getLength() < .001) {
		//					rightNodes.add(noderA.get(j));
		//			}
		//		}
		//	}		
		//}
		//if( rightNodes.size() == 0 ) {
		//	System.out.println("processImageChunkTest2 no right image nodes found for left image scan line at "+yStart);
		//	return;
		//}
		//int[] score = null;
		octree_t oscore = null;
		//double minscore = Double.MAX_VALUE;
		for( octree_t inode : leftNodes) {
			int iscore = 0;
			int nscore = 0;
			int yscore = 0;
			int zscore = 0;
			int tscore = 0;
			int isize = 0;
			double cScore = Double.MAX_VALUE;
			double dScore = Double.MAX_VALUE;
			int iScore = Integer.MAX_VALUE;
			int tScore = 0;
			synchronized(inode) {
			isize = inode.getIndexes().size();
			// check all right nodes against the ones on this scan line
			found = false;
			synchronized(noderA) {
				for(int j = 0; j < noderA.size(); j++) {
					double yDiff =  Math.abs(inode.getCentroid().y-noderA.get(j).getCentroid().y);
					if( yDiff <= yTolerance ) {
						if( !found ) { 
							found = true;
						}
						// found in range, acos of dot product
						double cscore = Math.acos(inode.getNormal2().and(noderA.get(j).getNormal2()));
						double tcscore = Math.abs(inode.getVariance2()-noderA.get(j).getVariance2());
						//double dscore = Math.acos(inode.getNormal3().and(noderA.get(j).getNormal3()));
						//if(DEBUGTEST2) {
						//	double tcscore = inode.getNormal2().multiplyVectorial(noderA.get(j).getNormal2()).getLength();	
						//	System.out.println("matchRegionsAssignDepth segment scores for centroid Y="+(yStart-(camheight/2))+" cross score ="+tcscore+" dot score ="+cscore+" part="+(inode.getNormal2().and(noderA.get(j).getNormal2()))+" ***** "+Thread.currentThread().getName()+" ***" );
						//}
						if(cscore < .1) {
							if(cscore < cScore) {
								cScore = cscore;
								dScore = tcscore;
								iScore = noderA.get(j).getIndexes().size();
								++yscore; // weedout when less
								oscore = noderA.get(j);
							} else {
								if(cscore == cScore) { // equal , match eigenvalue, then number points
									if( tcscore < dScore) {
										dScore = tcscore;
										iScore = noderA.get(j).getIndexes().size();
										++tscore; // weedout when less
										oscore = noderA.get(j);
									} else
									//if(DEBUGTEST2) {		
									//	System.out.println("matchRegionsAssignDepth segment scores for centroid Y="+(yStart-(camheight/2))+" normal2 score ="+cscore+" variance2 diff="+tcscore+" point diff="+Math.abs(isize-noderA.get(j).getIndexes().size())+" ***** "+Thread.currentThread().getName()+" ***" );
									//}
										if( tcscore == dScore && Math.abs(isize-noderA.get(j).getIndexes().size()) < Math.abs(isize-iScore) ) {
											iScore = noderA.get(j).getIndexes().size();
											++zscore; // weedout when equal
											oscore = noderA.get(j);
										} else
											++iscore; // points less matching
									/*
									int sum = 0;
									// all right nodes that qualified
									int lim = Math.min(inode.getIndexes().size(), noderA.get(j).getIndexes().size());
									synchronized(inode.getRoot().m_colors) {
										for(int c = 0; c < lim; c++) {
											Vector4d lcolor = inode.getRoot().m_colors.get(inode.getIndexes().get(c));
											Vector4d rcolor = noderA.get(j).getRoot().m_colors.get(noderA.get(j).getIndexes().get(c));				
											sum += Math.abs( (lcolor.x-rcolor.x) + (lcolor.y-rcolor.y) + (lcolor.z+rcolor.z) );
										}
									}
									if( sum < score) {
										++yscore;
										oscore = noderA.get(j);
										score = sum;
									}
									*/
								} else
									++iscore; // >, on plane, but no better
							}	
						} else
							++nscore; // off plane, beyond tolerance
						//++i1; // found or not, bump 'to' range	
					} else { // y out of tolerance
						if( !found ) { // y not found, y not <= , keep searching
							//++i1;
							continue;
						} else {
							break; // y was previously found, y now not <=, y will only increase
						}
					}
				} // right node loop
				if(!found) {
					if(DEBUGTEST2)
							System.out.println("matchRegionsAssignDepth left node centroid Y="+(yStart-(camheight/2))+", no right image nodes in tolerance ***** "+Thread.currentThread().getName()+" ***" );
					indexUnproc2.add(new IndexDepth(inode.getCentroid(), inode.getSize(), 0));
					continue; // next left node
				}
			} // right node array synch
			// tScore the number of actual assignments that occured
			tScore = yscore+zscore;
			// If we made no matches or we made multiple matches and we tried to match a vertical line
			// which removes our confidence factor
			if( tScore == 0 || (tScore > 1 && inode.getNormal2().x == 0 && inode.getNormal2().y == 1)) {
				if(DEBUGTEST2)
					System.out.println("matchRegionsAssignDepth rejection left node Y="+(yStart-(camheight/2))+" with "+tScore+" assignments, "+iscore+" failed assigments, "+nscore+" out of tolerance ***** "+Thread.currentThread().getName()+" ***");
				indexUnproc2.add(new IndexDepth(inode.getCentroid(), inode.getSize(), 0));
				continue;
			}
			} // inode synch
	
			// Option 2 continues..
			// now calculate the one closest to zero from sum of differences
			//int drank = Integer.MAX_VALUE;
			//int rank = 0;
			// move through array of differences we built in above scan loops
			//for(int s = 0; s < iscore; s++) {
			//	if (score[s] < drank) {
			//		rank = s;
			//		drank = score[s];
			//	}
			//}
			double xDiff = Math.abs(inode.getCentroid().x-oscore/*[rank]*/.getCentroid().x);
			double yDiff =  Math.abs(inode.getCentroid().y-oscore/*[rank]*/.getCentroid().y);
			if(DEBUGTEST2)
				if(tScore > 1) {
					System.out.println("matchRegionsAssignDepth WARNING left node Y="+(yStart-(camheight/2))+" got total of "+tScore+" assignments,"+iscore+" failed assignments, "+nscore+" out of tolerance, "+yscore+" better angle weedouts, "+tscore+" variance weedouts, "+zscore+" point match weedouts, resulting in node "+oscore+" ***** "+Thread.currentThread().getName()+" *** ");
				} else {
					System.out.println("matchRegionsAssignDepth left node Y="+(yStart-(camheight/2))+" got one good score, disparity="+xDiff+", "+iscore+" failed assignments, "+nscore+" out of tolerance, "+yscore+" better angle weedouts, "+tscore+" variance weedouts, "+zscore+" point match weedouts, resulting in node "+oscore+" ***** "+Thread.currentThread().getName()+" *** ");
				}
			//calc the disparity and insert into collection
			//we will call disparity the distance to centroid of right
			double pix = Bf/Math.hypot(xDiff, yDiff);	
			if( pix >=maxHorizontalSep) {
				if(DEBUGTEST2)
					System.out.println("matchRegionsAssignDepth left node at Y="+(yStart-(camheight/2))+" PIX TRUNCATED FROM:"+pix+" ***** "+Thread.currentThread().getName()+" *** ");
				pix = maxHorizontalSep;
				//if( pix < maxHorizontalSep) {
				// set points in octree cell to this disparity
				//for(int c = 0; c < inode.getIndexes().size(); c++) {
					//Vector4d tpoint = inode.getRoot().m_points.get(inode.getIndexes().get(c));
					//tpoint.z = pix;//(Bf/2) - pix ;
					//tpoint.w = -1; // mark it as having been set
					//synchronized(indexDepth2) {
					//		indexDepth2.add(new IndexDepth(inode.getMiddle(), inode.getSize(), pix));
					//}
				//}
				//if(DEBUGTEST2)
				//	System.out.println("processImageChunkTest2 node left="+inode+" set "+inode.getIndexes().size()+" points to "+pix+" for line "+yStart+" ***** "+Thread.currentThread().getName()+" *** ");
			} //else
			synchronized(indexDepth2) {
				indexDepth2.add(new IndexDepth(inode.getCentroid(), inode.getSize(), pix));
				//indexDepth2.add(new IndexDepth(inode.getCentroid(), inode.getNormal2(), inode.getVariance2(), inode.getNormal3(), inode.getVariance3(), pix));
			}
			//	if(DEBUGTEST2)
			//		System.out.println("processImageChunkTest2 node left="+inode+" of "+inode.getIndexes().size()+" points NOT SET "+pix+" out of tolerance for line "+yStart+" ***** "+Thread.currentThread().getName()+" *** ");
			if(DEBUGTEST2)
				System.out.println("matchRegionsAssignDepth left node Y="+(yStart-(camheight/2))+" should set set "+isize+" points to "+pix+" ***** "+Thread.currentThread().getName()+" *** ");
		} // left octree nodes

		if( SAMPLERATE )
			System.out.println("matchRegionAssignDepth left node Y="+(yStart-(camheight/2)) +" ***** "+Thread.currentThread().getName()+" *** "+(System.currentTimeMillis()-etime)+" ms.***");
	}
	/**
	 * Process the minimal nodes we collected in the last step against the maximal nodes we generated now.
	 * The indexDepth collection has verified minimal octree node middle point and edge size that we use to determine
	 * whether they fall in the maximal envelope. If so find the points in the maximal envelope that fall within the minimal one and
	 * assign the depth of the minimal region to those points. If the point in question falls within no
	 * minimal region, find the edge of the closes minimal region to the point and assign the depth of the minimal region
	 * to that point. This process can be repeated until all the enclosed minimal regions are
	 * all assigned, by clearing the octree, setting the minimum subdivide level one step higher (s_level), and then
	 * calling the subdivide method and getting the new node arraylist via get_nodes.
	 * @param yStart Index into left maximal node array
	 * @param nodelA contains maximal octree nodes
	 * @param indexDepth2 contains envelopes of minimal octree nodes previously computed
	 * @param maxEnv 
	 */
	private void findEnclosedRegionsSetPointDepth(int yStart, List<octree_t> nodelA, List<envInterface> indexDepth2, List<envInterface> maxEnv, List<envInterface> zeroEnc) {
		octree_t inode = null;
		//double tdepth = -1;
		//Vector4d iMiddle = null;
		//Vector4d iNorm1 = null;
		//Vector4d iNorm2 = null;
		//double iMag1, iMag2;
		IndexMax env;
		double[] denv;
		//double iSize;
		double smean, svariance, ssigma;
		// get first available octree node and process it for this thread
		inode = nodelA.get(yStart);
		synchronized(inode) {
			//iMiddle = inode.getMiddle();
			//iSize = inode.getSize();
			denv = getEnvelope(inode.getCentroid(), inode.getNormal2(), inode.getVariance2(), inode.getNormal3(), inode.getVariance3());
			env = new IndexMax(inode, denv, 0);
		}
		if(DEBUGTEST3)
			System.out.println("findEnclosedRegionsSetPointDepth node="+yStart+" xmin="+denv[0]+" ymin="+denv[1]+" xmax="+denv[2]+" ymax="+denv[3]+" ***** "+Thread.currentThread().getName()+" *** ");
		ArrayList<envInterface> enclosed = new ArrayList<envInterface>();
		
		// find all minimal nodes with an envelope inside this node from previous step
		synchronized(indexDepth2) {
			for(int j = 0; j < indexDepth2.size(); j++) {
				envInterface d = indexDepth2.get(j);
				//if(d.enclosedBy(iMiddle, iSize)) {
				if(d.enclosedBy(env)) {
					//indexDepth.remove(j);
					// if we are more than 2 standard deviations from the overall mean, dont use this
					if( d.getDepth() <= (mean+(sigma*2)) )
						enclosed.add(d); // save for later point assignment
				}
			}
			for(envInterface r : enclosed)
				indexDepth2.remove(r);
		}
		if(enclosed.isEmpty()) {
			if(DEBUGTEST3)
				System.out.println("findEnclosedRegionsSetPointDepth node="+yStart/*+" size="+iSize*/+" encloses 0 nodes of "+indexDepth2.size()+" ***** "+Thread.currentThread().getName()+" *** ");
			zeroEnc.add(env);
			// save to see if we overlap or enclose a smaller maximal region which has a depth assignment
			return;
		}
		smean = svariance = 0;
		for(envInterface ind : enclosed) {
				  smean += ind.getDepth();
		}
		smean /= (double)enclosed.size();
		// give our saved maximal envelope the mean depth for later when we try to assign depth to point in
		// maximal areas that enclosed 0 nodes
		env.setDepth(smean);
		// add this maximal envelope, later try to match those that enclosed 0 with those that got a depth
		maxEnv.add(env);
		//for(IndexDepth ind : enclosed) {
		//		  svariance += Math.pow((smean - ind.depth),2);
		//}
		//svariance /= (double)enclosed.size();
		//ssigma = Math.sqrt(svariance);
		if( DEBUGTEST3 )
			  System.out.println("findEnclosedRegionsSetPointDepth node="+yStart+" Mean="+smean/*+" variance="+svariance+" standard deviation="+ssigma*/+" ***** "+Thread.currentThread().getName()+" *** ");
		// done with indexDepth, process enclosed array for each point, assign depth
		if(ASSIGNPOINTS) {
		synchronized(inode) {
			synchronized(inode.getRoot().m_points) {
				// for each point in this maximal node
				for(int c = 0; c < inode.getIndexes().size(); c++) {
					//double cdist = Double.MAX_VALUE;
					Vector4d tpoint = inode.getRoot().m_points.get(inode.getIndexes().get(c));
					if(DEBUGTEST3)
						if(tpoint.z != 1.0)
							System.out.println("findEnclosedRegionsSetPointDepth node="+yStart+" WARNING point may have been assigned depth already, tpoint="+tpoint+" ***** "+Thread.currentThread().getName()+" *** ");
					boolean notAssigned = true;
					// for each minimal node that was enclosed by maximal, does target point lie within?
					for( envInterface d : enclosed ) {
						//System.out.println("processImageChunkTest3 node="+inode+" encloses "+d+" ***** "+Thread.currentThread().getName()+" *** ");
						if(d.encloses(tpoint)) {
							notAssigned = false;
							tpoint.z = d.getDepth();
							break;
						} //else {
							// see if this point is closer to a new envelope, then use that depth
							// we are not looking for a real point to measure to, just the sides of the envelope
							//double txmin = Math.abs(tpoint.x-d.xmin);
							//double txmax = Math.abs(tpoint.x-d.xmax);
							//double tymin = Math.abs(tpoint.y-d.ymin);
							//double tymax = Math.abs(tpoint.y-d.ymax);
							//txmin = Math.min(txmin,  txmax);
							//tymin = Math.min(tymin, tymax);
							//txmin = Math.min(txmin,  tymin);
							//if( txmin < cdist ) {
							//	cdist = txmin; // distance to edge
							//	tdepth = d.depth; // depth of cell we assign to target
							//}
						//}
					}
					// did we assign the target point in maximal node to one of the minimal nodes inside?
					if( notAssigned ) {
						if(DEBUGTEST3)
							System.out.println("findEnclosedRegionsSetPointDepth node "+yStart+"="+inode+" point="+tpoint+" not enclosed by any "+enclosed.size()+" minimal areas, assigning default depth "+smean/*tdepth*/+" ***** "+Thread.currentThread().getName()+" *** ");
						tpoint.z = smean;//tdepth;
					} else 
						if(DEBUGTEST3)
							System.out.println("findEnclosedRegionsSetPointDepth node "+yStart+"="+inode+" point="+tpoint+" assigned depth "+tpoint.z+" ***** "+Thread.currentThread().getName()+" *** ");		
				}
				
			}
		}
		}

		if(DEBUGTEST3)
			System.out.println("findEnclosedRegionsSetPointDepth Maximal Envelope "+yStart+" of "+nodelA.size()+" node="+inode+" exits with "+indexDepth2.size()+" minimal envelopes ***** "+Thread.currentThread().getName()+" *** ");
	}
	/**
	 * Find the maximal regions that enclosed no minimal regions. Try to find another maximal region that overlaps
	 * partially or fully that has a mean depth
	 * we can use to assign depths to points in the zero enclosing region. We are ratcheting up the coplanar areas this way.
	 * @param yStart
	 * @param env Maximal regions that have a mean depth from last step
	 * @param zenc zero enclosing regions whose points need assignment from last step
	 */
	private void findZeroEnclosedSetPointDepth(int yStart, List<envInterface> env, List<envInterface> zenc) {
		envInterface senv = zenc.get(yStart);
		octree_t inode = ((IndexMax)senv).node;
		double smean = 0;
		ArrayList<envInterface> enclosed = new ArrayList<envInterface>();
		synchronized(env) {
			// try all maximal for this zero encloser, if we find one, use it and bail
			for(envInterface e: env) {
				if( e.enclosedBy(senv) || senv.enclosedBy(e)) {
					if(DEBUGTEST4) {
						if( e.enclosedBy(senv) )
							System.out.println("findZeroEnclosedSetPointDepth node="+yStart+" main envelope "+e+" enclosed by zero "+senv+" ***** "+Thread.currentThread().getName()+" *** ");
						else
							System.out.println("findZeroEnclosedSetPointDepth node="+yStart+" zero envelope "+senv+" enclosed by main "+e+" ***** "+Thread.currentThread().getName()+" *** ");
					}
					smean += e.getDepth();
					enclosed.add(e);
				}
			}
		}
			if( !enclosed.isEmpty()) {
				smean /= (double)enclosed.size();
					//senv.setDepth(e.getDepth());
				if(ASSIGNPOINTS) {
					synchronized(inode) {
					synchronized(inode.getRoot().m_points) {
						// for each point in this maximal node
						for(int c = 0; c < inode.getIndexes().size(); c++) {
							Vector4d tpoint = inode.getRoot().m_points.get(inode.getIndexes().get(c));
							if(tpoint.z != 1.0) {
								if(DEBUGTEST4)
									System.out.println("findZeroEnclosedSetPointDepth node="+yStart+" WARNING point has depth already, point="+tpoint+" ***** "+Thread.currentThread().getName()+" *** ");
							} else {
								// find the envelope this point lie in, if none, give it the mean of all regions
								boolean notAssigned = true;
								// for each minimal node that was enclosed by maximal, does target point lie within?
								for( envInterface d : enclosed ) {
									//System.out.println("processImageChunkTest3 node="+inode+" encloses "+d+" ***** "+Thread.currentThread().getName()+" *** ");
									if(d.encloses(tpoint)) {
										notAssigned = false;
										tpoint.z = d.getDepth();
										if(DEBUGTEST4)
											System.out.println("findZeroEnclosedSetPointDepth node "+yStart+"="+inode+" point="+tpoint+" set from env="+d+" from "+enclosed.size()+" envelopes. ***** "+Thread.currentThread().getName()+" *** ");	
										break;
									} //else {
								}
								if(notAssigned) {
									tpoint.z = smean;//e.getDepth();
									if(DEBUGTEST4)
										System.out.println("findZeroEnclosedSetPointDepth node "+yStart+"="+inode+" point="+tpoint+" set from mean, none found in "+enclosed.size()+" envelopes. ***** "+Thread.currentThread().getName()+" *** ");
								}
							}
						}
					}
					}
					return;
				}
			}
		//} synchronized env
		if(DEBUGTEST4)
			System.out.println("findZeroEnclosedSetPointDepth node "+yStart+"="+inode+" found no valid alternate envelope ***** "+Thread.currentThread().getName()+" *** ");	
	}
	/**
	 * Transform the coord at x,y by the Matrix3 transform provided, translate and rotate.
	 * @param x
	 * @param y
	 * @param transform
	 * @return the transformed coords
	 */
	private int[] scale(int x, int y, Matrix3 transform) {
		int ks, ms;
		// transform to 3D plane offsets
		if( x <= camWidth/2 )
			ks = (camWidth/2) - x;
		else
			ks = x - (camWidth/2);
		if( y <= camHeight/2 )
				ms = (camHeight/2) - y;
			else
				ms = y - (camHeight/2);
		Point t = new Point(ks, ms, 0, Color.red );
		Vertex v1 = transform.transform(t);
		// transform back to 2D plane
		if( x <= camWidth/2 )
			ks = (camWidth/2) - (int)v1.x;
		else
			ks = (int)v1.x + (camWidth/2);
		if( y <= camHeight/2 )
			ms = (camHeight/2) - (int)v1.y;
		else
			ms = (int)v1.y + (camHeight/2);
		if( ks < 0 || ks >= camWidth || ms < 0 || ms >=camHeight)
			System.out.println("Coords transformed out of range:"+ks+","+ms+" from:"+x+","+y);
		// ks and ms should now be transformed to coord of right image that slides to left
		return new int[]{ks,ms};
		
	}

	
	/**
	 * Plot a course based on visual data
	 * @param simage
	 * @param rangeL
	 * @param rangeR
	 */
	public void navigate(short[][][] simage, int rangeL, int rangeR) {
		int aveL, aveC, aveR, aveLN, aveCN, aveRN;
		//for all the pixels p of the central window {
		//	if D(p) > T {
		//		counter++ 
		//	}
		//	numC++ 
		//}
		// center from rangeL to rangeR
		aveC = 0;
		aveCN = 0;
		for(int i = rangeL; i< rangeR; i++)	{
			for(int j = 0; j < simage[0].length; j++) {
				aveC += simage[i][j][3];
				++aveCN;
			}
		}
		aveC /= aveCN;
		// left part from 0 to rangeL
		aveL = 0;
		aveLN = 0;
		for(int i = 0; i < rangeL; i++)	{
			for(int j = 0; j < simage[0].length; j++) {
				aveL += simage[i][j][3];
				++aveLN;
			}
		}
		//	for all the pixels p of the left window {
		//		sumL =+ D(p)
		//		numL++ 
		//	}
		aveL /= aveLN;
		//	for all the pixels p of the right window {
		//		sumR =+ D(p)
		//		numR++ 
		//	} 
		//}
		// right from rangeR to end
		aveR = 0;
		aveRN = 0;
		for(int i = rangeR; i < simage.length; i++)	{
			for(int j = 0; j < simage[0].length; j++) {
				aveR += simage[i][j][3];
				++aveRN;
			}
		}
		aveR /= aveRN;
	//if counter < r% of numC {
	//	GO STRAIGHT 
	//} else {	
	//avL = sumL / numL
	//avR = sumR / numR
	//if avL <avR {
	//	GO LEFT 
	//} else {
	//GO RIGHT 
	//} 
	//}
		System.out.println("AVE:="+aveL+","+aveC+","+aveR);
		if( aveL < aveR && aveL < aveC )
			System.out.println("<<GO LEFT<<");
		else
			if( aveR < aveC && aveR < aveL)
				System.out.println(">>GO RIGHT>>");
			else
				System.out.println("||GO STRAIGHT||");
	}
	
		
	   public static Color getShade(Color color, double shade) {
	        double redLinear = Math.pow(color.getRed(), 2.4) * shade;
	        double greenLinear = Math.pow(color.getGreen(), 2.4) * shade;
	        double blueLinear = Math.pow(color.getBlue(), 2.4) * shade;

	        int red = (int) Math.pow(redLinear, 1/2.4);
	        int green = (int) Math.pow(greenLinear, 1/2.4);
	        int blue = (int) Math.pow(blueLinear, 1/2.4);
	        if( red < 0 ) {
	        	System.out.println("Red out of range="+red);
	        	red = 0;
	        }
	        if( red > 255) {
	        	System.out.println("Red out of range="+red);
	        	red = 255;
	        }
	        if( green < 0 ) {
	        	System.out.println("Green out of range="+green);
	        	green = 0;
	        }
	        if( green > 255) {
	        	System.out.println("Green out of range="+green);
	        	green = 255;
	        }
	        if( blue < 0 ) {
	        	System.out.println("Blue out of range="+blue);
	        	blue = 0;
	        }
	        if( blue > 255) {
	        	System.out.println("Blue out of range="+blue);
	        	blue = 255;
	        }
	        return new Color(red, green, blue);
	    }
	    /*
	    public static List<Triangle> inflate(List<Triangle> tris) {
	        List<Triangle> result = new ArrayList<>();
	        for (Triangle t : tris) {
	            Vertex m1 = new Vertex((t.v1.x + t.v2.x)/2, (t.v1.y + t.v2.y)/2, (t.v1.z + t.v2.z)/2);
	            Vertex m2 = new Vertex((t.v2.x + t.v3.x)/2, (t.v2.y + t.v3.y)/2, (t.v2.z + t.v3.z)/2);
	            Vertex m3 = new Vertex((t.v1.x + t.v3.x)/2, (t.v1.y + t.v3.y)/2, (t.v1.z + t.v3.z)/2);
	            result.add(new Triangle(t.v1, m1, m3, t.color));
	            result.add(new Triangle(t.v2, m1, m2, t.color));
	            result.add(new Triangle(t.v3, m2, m3, t.color));
	            result.add(new Triangle(m1, m2, m3, t.color));
	        }
	        for (Triangle t : result) {
	            for (Vertex v : new Vertex[] { t.v1, t.v2, t.v3 }) {
	                double l = Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z) / Math.sqrt(30000);
	                v.x /= l;
	                v.y /= l;
	                v.z /= l;
	            }
	        }
	        return result;
	    }
	    */
	
	class Vertex {
	    double x;
	    double y;
	    double z;
	    Vertex(double x, double y, double z) {
	        this.x = x;
	        this.y = y;
	        this.z = z;
	    }
		public String toString() {
			return "x="+x+" y="+y+" z="+z;
		}
	}

	class Point extends Vertex {
		Color color;
		Point(double x, double y, double z, Color color) {
			super(x, y, z);
			this.color = color;
		}
		Point(Vertex v, Color color) {
			super(v.x, v.y, v.z);
			this.color = color;
		}
		public String toString() {
			return super.toString()+" color:"+color;
		}
	}

	class Triangle {
	    Vertex v1;
	    Vertex v2;
	    Vertex v3;
	    Color color;
	    Triangle(Vertex v1, Vertex v2, Vertex v3, Color color) {
	        this.v1 = v1;
	        this.v2 = v2;
	        this.v3 = v3;
	        this.color = color;
	    }
	}

	class Matrix3 {
	    double[] values;
	    Matrix3(double[] values) {
	        this.values = values;
	    }
	    Matrix3 multiply(Matrix3 other) {
	        double[] result = new double[9];
	        for (int row = 0; row < 3; row++) {
	            for (int col = 0; col < 3; col++) {
	                for (int i = 0; i < 3; i++) {
	                    result[row * 3 + col] +=
	                        this.values[row * 3 + i] * other.values[i * 3 + col];
	                }
	            }
	        }
	        return new Matrix3(result);
	    }
	    Vertex transform(Vertex in) {
	        return new Vertex(
	            in.x * values[0] + in.y * values[3] + in.z * values[6],
	            in.x * values[1] + in.y * values[4] + in.z * values[7],
	            in.x * values[2] + in.y * values[5] + in.z * values[8]
	        );
	    }
	    public String toString() {
	    	return String.format("[%15.8f,%15.8f,%15.8f]\r\n[%15.8f,%15.8f,%15.8f]\r\n[%15.8f,%15.8f,%15.8f]\r\n",values[0],values[1],values[2],values[3],values[4],values[5],values[6],values[7],values[8]);
	    	
	    }
	}
	   /**
	    * Horizontal and vertial FOV considered the same, in radians.
	    * @param x
	    * @param y
	    * @param depth
	    * @param width
	    * @param height
	    * @param FOV Camera field of view in radians
	    * @return
	    */
		public Vertex depthToVertex(int x, int y, int depth, int width, int height, double FOV) {
			double vx,vy,vz;
		    //calculate x-coordinate
		    double alpha_h = (Math.PI - FOV) / 2;
		    double gamma_i_h = alpha_h + x*(FOV / width);
		    vx = depth / Math.tan(gamma_i_h);
		    //calculate y-coordinate
		    double alpha_v = 2 * Math.PI - FOV / 2;
		    double gamma_i_v = alpha_v + y*(FOV / width);
		    vy = depth * Math.tan(gamma_i_v)*-1;
		    //z-coordinate
		    vz = depth;
		    return new Vertex(vx,vy,vz);
		}
		/**
		 * The envelope of the vector of centroid cent with magnitude eigenvalue plus and minus vmag1,
		 * in the direction of eigenvector norm1, to the vector of centroid to norm2 of length plus and minus vmag2. Since
		 * eigenvectors are represented by normal vectors, we scale them to centroid, with angle eigenvector,
		 * magnitude eigenvalue plus and eigenvalue minus. So its a cross centered on centroid with the axis in the direction of
		 * the 2 normal vectors, perpendicular to each other of course, with the coordinates of centroid plus and minus the 2 magnitudes, 
		 * of which the envelope min and max x and y are taken. When we use the spherical_to_cartesian it gives us a 3D
		 * point even though we are concentrating on getting a 2D envelope, and the axis are oriented orthogonal to the 2D
		 * point cloud, so we have to take the maximum Z offset and 'rotate' that to the x axis. We rotate because the x 
		 * coordinates of our 2 generated cross axis are always the same as they are oriented perpendicular to the plane 
		 * of the image.
		 * @param cent centroid of envelope
		 * @param norm1 normal vector of first axis
		 * @param vmag1 magnitude of first normal vector
		 * @param norm2 normal vector of second axis
		 * @param vmag2 magnitude of second normal vector
		 * @return xmin, ymin, xmax, ymax, of envelope of generated cross from passed parameters
		 */
		public static double[] getEnvelope(Vector4d cent, Vector4d norm1, double vmag1, Vector4d norm2, double vmag2) {
			Vector4d cen1 = new Vector4d();
			Vector4d cen2 = new Vector4d();
			Vector4d cen3 = new Vector4d();
			Vector4d cen4 = new Vector4d();
			double[] tpr = octree_t.cartesian_to_spherical(norm1);
			// mid axis normal2
			octree_t.spherical_to_cartesian(cen1, cent, tpr[0], tpr[1], tpr[2], -vmag1);
			// generate point at end of segment
			octree_t.spherical_to_cartesian(cen2, cent, tpr[0], tpr[1], tpr[2], vmag1);
			//System.out.println("IndexDepth enclosedBy cen1="+cen1+" cen2="+cen2+" vmag1="+vmag1+" tpr:"+tpr[0]+" "+tpr[1]+" "+tpr[2]);
			// long axis
			tpr = octree_t.cartesian_to_spherical(norm2);
			// subtract length of longest axis normal3
			octree_t.spherical_to_cartesian(cen3, cent, tpr[0], tpr[1], tpr[2], -vmag2);
			// generate point at end of long segment
			octree_t.spherical_to_cartesian(cen4, cent, tpr[0], tpr[1], tpr[2], vmag2);
			// find greatest z diff, make that x
			double zdiff = Math.max(Math.abs(cent.z-cen4.z), Math.max(Math.abs(cent.z-cen3.z),  
					Math.max(Math.abs(cent.z-cen1.z), Math.abs(cent.z-cen2.z))));
			//System.out.println("IndexDepth enclosedBy cen3="+cen3+" cen4="+cen4+" vmag2="+vmag2+" tpr:"+tpr[0]+" "+tpr[1]+" "+tpr[2]);
			double txmin = Math.min(cen4.x-zdiff,Math.min(cen3.x-zdiff, Math.min(cen1.x-zdiff, cen2.x-zdiff)));
			double txmax = Math.max(cen4.x+zdiff,Math.max(cen3.x+zdiff, Math.max(cen1.x+zdiff, cen2.x+zdiff)));
			double tymin = Math.min(cen4.y,Math.min(cen3.y, Math.min(cen1.y, cen2.y)));
			double tymax = Math.max(cen4.y,Math.max(cen3.y, Math.max(cen1.y, cen2.y)));
			if(DEBUG)
				System.out.println("getEnvelope txmin="+txmin+" tymin="+tymin+" txmax="+txmax+" tymax="+tymax);
			return new double[]{txmin, tymin, txmax, tymax};
		}
		
		static public interface envInterface {
			public double[] getEnv();
			public double getDepth();
			public boolean encloses(Vector4d tpoint);
			public boolean encloses(envInterface e);
			public boolean enclosedBy(envInterface e);
			public void setDepth(double depth);
		}
		/**
		 * Abstraction of processed envelopes with the addition of depth. Middle typically refers to centroid
		 * of point cloud in octree node. size is typically derived from octree node edge length, but can
		 * also include the length of the vector of centroid with magnitude eigenvalue in direction eigenvector, since
		 * eigenvectors are represented by normal vectors, we scale them to centroid with angle eigenvecotr magnitude eigenvalue plus
		 * and eigenvalue minus.
		 * @author jg
		 *
		 */
		final class IndexDepth implements envInterface {
			public Vector4d middle;
			public double xmin, ymin, xmax, ymax;
			public double depth;
			public IndexDepth(Vector4d middle, double m_size, double depth) {
				this.middle = middle;
				xmin = middle.x-(m_size/2);
				xmax = middle.x+(m_size/2);
				ymin = middle.y-(m_size/2);
				ymax = middle.y+(m_size/2);
				this.depth = depth;
			}
			/**
			 * does the passed square lie within this one edges included?
			 * @param tmiddle
			 * @param tm_size
			 * @return
			 */
			public boolean encloses(Vector4d tmiddle, double tm_size) {
				double txmin = tmiddle.x-(tm_size/2);
				double tymin = tmiddle.y-(tm_size/2);
				double txmax = tmiddle.x+(tm_size/2);
				double tymax = tmiddle.y+(tm_size/2);
				return (xmin <= txmin && xmax >= txmax && ymin <= tymin && ymax >= tymax);
			}
			/**
			 * does the passed square lie within this one edges included?
			 * @param tmiddle
			 * @param tm_size
			 * @return
			 */
			@Override
			public boolean encloses(envInterface tenv) {
				return (xmin <= tenv.getEnv()[0] && xmax >= tenv.getEnv()[2] && ymin <= tenv.getEnv()[1] && ymax >= tenv.getEnv()[3]);
			}
			/**
			 * does the passed point lie within this envelope edges included?
			 * @param tmiddle
			 * @param tm_size
			 * @return
			 */
			@Override
			public boolean encloses(Vector4d point) {
				return (xmin <= point.x && xmax >= point.x && ymin <= point.y && ymax >= point.y);
			}
			/**
			 * does 'this' lie within the passed square?
			 * @param tmiddle
			 * @param tm_size
			 * @return
			 */
			public boolean enclosedBy(Vector4d tmiddle, double tm_size) {
				double txmin = tmiddle.x-(tm_size/2);
				double tymin = tmiddle.y-(tm_size/2);
				double txmax = tmiddle.x+(tm_size/2);
				double tymax = tmiddle.y+(tm_size/2);
				return (txmin <= middle.x && txmax >= middle.x && tymin <= middle.y && tymax >= middle.y);
			}
			/**
			 * 
			 * @param tenv txmin,tymin,txmax,tymax
			 * @return
			 */
			@Override
			public boolean enclosedBy(envInterface tenv) {
				//System.out.println("IndexDepth enclosedBy txmin="+txmin+" middle.x="+middle.x+" txmax="+txmax+" tymin="+tymin+" middle.y="+middle.y+" tymax="+tymax);
				return (tenv.getEnv()[0]/*txmin*/ <= middle.x && tenv.getEnv()[2]/*txmax*/ >= middle.x && 
						tenv.getEnv()[1]/*tymin*/ <= middle.y && tenv.getEnv()[3]/*tymax*/ >= middle.y);
			}
			@Override
			public boolean equals(Object tind) {
				return ( xmin == ((envInterface)tind).getEnv()[0] && xmax == ((envInterface)tind).getEnv()[2] &&
						 ymin == ((envInterface)tind).getEnv()[1] && ymax == ((envInterface)tind).getEnv()[3]);
			}
			@Override
			public String toString() {
				return "IndexDepth xmin="+xmin+" ymin="+ymin+" xmax="+xmax+" ymax="+ymax+" depth="+depth+" middle="+middle;
			}
			@Override
			public double[] getEnv() {
				return new double[]{xmin,ymin,xmax,ymax};
			}
			@Override
			public double getDepth() {
				return depth;
			}
			@Override
			public void setDepth(double depth) {
				this.depth = depth;		
			}
			
		}
		final class IndexMax implements envInterface {
			public octree_t node;
			public double xmin, ymin, xmax, ymax;
			public double depth;
			public IndexMax(octree_t node, double[] env, double depth) {
				this.node = node;
				xmin = env[0];
				xmax = env[2];
				ymin = env[1];
				ymax = env[3];
				this.depth = depth;
			}
			/**
			 * does the passed square lie within this one, even partially?
			 * is xmin of this between xmin and xmax of passed and ymin of
			 * this between ymin and ymax of passed OR
			 * xmax of this between xmin and xmax of passed and ymax of this
			 * between ymin and ymax of passed
			 * @param env
			 * @return true if passed envelope lies within thisone
			 */
			@Override
			public boolean encloses(envInterface env) {
				return ((xmin >= env.getEnv()[0] && xmin <= env.getEnv()[2] &&
						 ymin >= env.getEnv()[1] && ymin <= env.getEnv()[3]) || 
						(xmax >= env.getEnv()[0] && xmax <= env.getEnv()[2] &&
						 ymin >= env.getEnv()[1] && ymin <= env.getEnv()[3]) ||
						(xmax >= env.getEnv()[0] && xmax <= env.getEnv()[2] &&
						 ymax >= env.getEnv()[1] && ymax <= env.getEnv()[3]) ||
						(xmin >= env.getEnv()[0] && xmin <= env.getEnv()[2] &&
						 ymax >= env.getEnv()[1] && ymax <= env.getEnv()[3]) );
			}
			/**
			 * does the passed point lie within this envelope edges included?
			 * @param tmiddle
			 * @param tm_size
			 * @return
			 */
			@Override
			public boolean encloses(Vector4d point) {
				return (xmin <= point.x && xmax >= point.x && ymin <= point.y && ymax >= point.y);
			}
			/**
			 * does 'this' lie within the passed square?
			 * @param tmiddle
			 * @param tm_size
			 * @return
			 */
			@Override
			public boolean enclosedBy(envInterface env) {
				return ((env.getEnv()[0] >= xmin && env.getEnv()[0] <= xmax &&
						 env.getEnv()[1] >= ymin && env.getEnv()[1] <= ymax) || 
						(env.getEnv()[2] >= xmin && env.getEnv()[2] <= xmax &&
						 env.getEnv()[1] >= ymin && env.getEnv()[1] <= ymax) ||
						(env.getEnv()[2] >= xmin && env.getEnv()[2] <= xmax &&
						 env.getEnv()[3] >= ymin && env.getEnv()[3] <= ymax) ||
						(env.getEnv()[0] >= xmin && env.getEnv()[0] <= xmax &&
						 env.getEnv()[3] >= ymin && env.getEnv()[3] <= ymax) );
			}
	
			@Override
			public boolean equals(Object tind) {
				return ( xmin == ((envInterface)tind).getEnv()[0] && xmax == ((envInterface)tind).getEnv()[2] &&
						 ymin == ((envInterface)tind).getEnv()[1] && ymax == ((envInterface)tind).getEnv()[3]);
			}
			@Override
			public String toString() {
				return "IndexMax xmin="+xmin+" ymin="+ymin+" xmax="+xmax+" ymax="+ymax+" depth="+depth+" node="+node;
			}
			@Override
			public double[] getEnv() {
				return new double[]{xmin,ymin,xmax,ymax};
			}
			@Override
			public double getDepth() {
				return depth;
			}
			@Override
			public void setDepth(double depth) {
				this.depth = depth;
			}
			
		}
		
		public static void main(String[] args) throws Exception {
			VideoProcessor vp = new VideoProcessor();
			vp.spinUp();
			BufferedImage imageL1 = ImageIO.read(new File(vp.outDir+"/"+args[0]));
			BufferedImage imageR1 = ImageIO.read(new File(vp.outDir+"/"+args[1]));
			vp.queueL.addLast(imageL1);
			vp.queueR.addLast(imageR1);
			
		}

}


