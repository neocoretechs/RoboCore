package com.neocoretechs.robocore;

import java.awt.Color;
import java.awt.geom.Path2D;
import java.awt.geom.PathIterator;
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
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.Iterator;
import java.util.List;
import java.util.Map.Entry;
import java.util.SortedMap;
import java.util.TreeMap;
import java.util.TreeSet;
import java.util.concurrent.BrokenBarrierException;
import java.util.concurrent.CyclicBarrier;
import java.util.concurrent.atomic.AtomicInteger;

import javax.imageio.ImageIO;

import org.jtransforms.dct.DoubleDCT_1D;
import org.jtransforms.dct.FloatDCT_1D;
import org.jtransforms.dct.FloatDCT_2D;
import org.jtransforms.utils.IOUtils;
import org.ros.concurrent.CircularBlockingDeque;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

//import com.neocoretechs.machinevision.CannyEdgeDetector;
import com.neocoretechs.machinevision.MeanColorGenerator;
//import com.neocoretechs.machinevision.ParallelCannyEdgeDetector;
import com.neocoretechs.machinevision.PolyBezierPathUtil;
import com.neocoretechs.machinevision.hough3d.Vector4d;
import com.neocoretechs.machinevision.hough3d.hough_settings;
import com.neocoretechs.machinevision.hough3d.octree_t;
import com.neocoretechs.machinevision.hough3d.writer_file;
import com.neocoretechs.robocore.machine.bridge.RadixTree;

/**
 * Create a disparity map for the left and right images taken from stereo cameras published to bus.
 * We break the image into  bands and assign each band to a processing thread. 
 * We have an option to write files that can be read by CloudCompare for testing.
 *  __mode:= option on commandline determines how we process, such as 'edge','display', 'display3D',etc.
 *  
 * The new algorithm for stereo ChromoSpatial Coplanar Region (CHROMOSCARM) Matching. This fast algorithm for
 * assigning depth to stereo point clouds works well on low
 * resolution images, requires no calibration of cameras, and is immune to misalignment of the cameras. 
 * 
 * New algorithm for stereo matching:
 * (The terms region, node, and octree node and octree cell are synonymous)
 * The steps are:
 * 1.) Create the ChromoSpatial model from a pseudo 3D representation of image X,Y,Z coords with the Z axis as 
 * the color value, sans alpha, converted to greyscale and subtracted from the mean greyscale value. 
 *
 * 2.) Generate 2 octrees of model converted images at the minimal node level, now 7. this step in parallel by assigning
 * the octree build to one Y image scan line per thread.
 *
 * 3.) Use PCA on the 2 images to find minimal coplanar regions in both octrees and generate eigenvectors 
 * and eigenvalues of the principal axis of the minimal coplanar regions. 
 * The octree subdivision scheme relies on hough_settings.min_isotropy to set how much variance from PCA eigenvalues
 * variance2/variance3 allowed when deciding whether to subdivide the octree at each level. 
 * This step in a single thread after barrier 
 * synchronization of the build step. Multiple threads not necessary as this step is fast.
 *
 * 4.) Process the left image coplanar regions against the right coplanar regions by comparing 
 * the cosine of dot products of the eigenvectors second axis and the magnitude of the eigenvalue
 * for all minimal right regions in the yTolerance of the left Y scan line assigned to the thread calling the method. 
 * Match the regions at minimal level from left to right image. Weed out the ones beyond yTolerance
 * and match the arccosine of vector dot product of the secondary eigenvector if the difference between the left
 * node eigenvectors and right is below tolerance. From the candidates below tolerance, select the one
 * that has the minimum difference in eigenvector dot. Where they are the same compare eigenvalue and take the
 * least difference. Finally, all things being equal count the points in the region and find closest. In the case of vertical
 * lines with multiple matches, reject the match, if one match to a vertical line, then use that.
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
 * 11.) Regenerate coplanar regions via PCA  at a higher octree level, using level 5, if there are unprocessed smaller
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
	private static final boolean DEBUG = true;
	private static final boolean DEBUGTEST3 = false;
	private static final boolean DEBUGTEST2 = false;
	private static final boolean DEBUGTEST4 = false;
	private static final boolean SAMPLERATE = false; // display thread timing performance data
	private static final boolean TIMER = true; // display global performance data
	private static final boolean EDGETIMER = false; // display edge detect performance data
	private static final boolean QUEUETIMER = false; // display frames queued per second
	private static final boolean WRITEFILES = false; // write full set of display files
	private static final boolean WRITEGRID = false; // write occupancy grid derived from depth points
	private static final boolean WRITEPLANARS = false; // write left and right minimal planars
	private static final boolean WRITEPLANES = false; // write final planes approximations
	private static final boolean WRITEUNCORR = false; // write uncorrelated minimal planars
	private static final boolean WRITECORR = true; // write correlated minimal planar envelopes
	private static final boolean WRITEZEROENC = false; // write maximal envelopes enclosing no minimal planar regions
	private static final boolean WRITEENCZERO = false; // write minimal planar regions enclosed by no maximal envelope
	private static final boolean WRITEMODEL = false; // write left and right model, if ASSIGNPOINTS true left file has 3D
	private static final boolean WRITEJPEGS = false; // write left right raw images from stereo bus
	private static final boolean WRITERMS = true; // write an edge file with name RMSxxx (where xxx is rms value) when RMS value changes
	private static final int WRITEMATCHEDPAIRS = 0; // if > 0, the number of individual matched pair files to write from the first number matched to check matching
	private static final boolean ASSIGNPOINTS = false; // assign depth to points in file vs simply compute planars
	private static final boolean SMOOTHEGRID = false; // Bezier smoothing of occupancy nav grid

    private BufferedImage imageL = null;
    private BufferedImage imageR = null;
    private BufferedImage imageLx = null;
    private BufferedImage imageRx = null;
	private int[] dataL = null; // left array return from model generation with Z magnitudes
	private int[] dataR = null; // right model array with magnitudes
	private int[] dataLp = null; // prev left array return from model gen with magnitudes
	private int[] dataRp = null; // prev right canny array with magnitudes

	int outWidth = 640;
	int outHeight = 480;
     
	String mode = "";
	String outDir = "/users/jg/workspace/robocore/motionclouds";
	int frames = 0;
	static int files = 0;
	// optical parameters
    final static float f = 4.4f; // focal length mm
    final static float B = 100.0f; // baseline mm
    final static float FOVD = 60; // degrees field of view
    final static double FOV = 1.04719755; // radians FOV
    final static int camWidth = 640; // pixels
    final static int camHeight = 480;
    final static int leftBound = -207;
    final static int rightBound = 207;
    final static double fp = B*(camWidth*.5)/Math.tan(FOV *.5 * (Math.PI/2)); //focal length in pixels
    final static double Bf = B*f;// calc the Bf of Bf/d
    final static int maxHorizontalSep = (int) (Bf-1)/4; // max pixel disparity
    final static int yTolerance = 100; // pixel diff in y of potential candidates
    // .0174533 rad = 1 deg
    final static double aTolerance = .007;// minimum angle between normals eigenvector axis 1
    final static double bTolerance = .5;// max difference between eigenvalues, eigenvector axis 1
    final static int cTolerance = 10; // eigenvector eigenvalue axis 2
    final static int dTolerance = 10; // eigenvector eigenvalue axis 3
    public final static int imageIntegrate = 0; // millis to accumulate images and integrate edge detects
    final static int corrSize = 50;
    final static int corrHalfSize = 25;
    final static double DCTRMSSigma = 1.35; // number of standard deviations from DCT RMS mean before we decide its a new image

    CircularBlockingDeque<Object[]> queueI = new CircularBlockingDeque<Object[]>(10);
	
	private int sequenceNumber,lastSequenceNumber;
	long time1;

	BufferedImage bimage = null;
	octree_t node = null;
	octree_t nodel = null;
	octree_t noder = null;
	List<envInterface> indexDepth; // correlated and matched minimal regions
	List<envInterface> indexUnproc; // uncorrelated minimal regions
	List<envInterface> maxEnv; // maximal regions that enclose one or more minimal regions
	List<envInterface> zeroEnc; // maximal regions that enclose zero minimal regions
	List<int[]> leftYRange; // from/to position in array for sorted Y centroid processing

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
	double[] prevDCT = null;
	int pairCount;
	int metric1 = 0; int metric2 = 0; int metric2b; int metric3 = 0; int metric4 = 0; int metric5 = 0;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("subs_videoproc");
	}
	
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		//Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
		System.out.println("Processing "+camWidth+" by "+camHeight);
		// bring up all parallel  processing threads
		spinUp();
		// spin the model generator class and threads
		final PixelsToModel modelGen = new PixelsToModel();
		modelGen.spinGen();
		// subscribe to stereo image bus to start flow of images into queue
		final Subscriber<stereo_msgs.StereoImage> imgsub =
				connectedNode.newSubscriber("/stereo_msgs/StereoImage", stereo_msgs.StereoImage._TYPE);
		/**
		 * Image extraction from bus, then image processing.
		 */
		imgsub.addMessageListener(new MessageListener<stereo_msgs.StereoImage>() {
		@Override
		public void onNewMessage(stereo_msgs.StereoImage img) {
			BufferedImage imageL1 = null;
			BufferedImage imageR1 = null;
			ByteBuffer cbL, cbR;
			byte[] bufferL;
			byte[] bufferR;
			long slew = System.currentTimeMillis() - time1;
			// signal commencement processing by placing the last in queue
			try {
					cbL = img.getData();
					bufferL = cbL.array();
					InputStream in = new ByteArrayInputStream(bufferL);
					imageL1 = ImageIO.read(in);
					in.close();
					cbR = img.getData2();
					bufferR = cbR.array();
					in = new ByteArrayInputStream(bufferR);
					imageR1 = ImageIO.read(in);
					in.close();
					//
					modelGen.imageL1 = imageL1;
					modelGen.imageR1 = imageR1;
					modelGen.latchOutL.reset();
					modelGen.latchOutR.reset();
					modelGen.latchL.reset();
					modelGen.latchR.reset();
					long edgeTime = System.currentTimeMillis();
					try {
						modelGen.latchL.await();
						modelGen.latchR.await();
					} catch (InterruptedException | BrokenBarrierException e) {System.out.println("<<Unexpected InLatch barrier break>>");}
					try {
						modelGen.latchOutL.await();
						modelGen.latchOutR.await();
					} catch (InterruptedException | BrokenBarrierException e) {System.out.println("<<Unexpected OutLatch barrier break>>");}
					if(EDGETIMER)
						System.out.println("Edge generator time="+(System.currentTimeMillis()-edgeTime)+" ms.");
	        	    // We have a set amount of time to resolve our view then send it down the line
					if( slew >= imageIntegrate ) {
						queueI.addLast(new Object[]{imageL1,imageR1,dataLp,dataRp});
						dataLp = null;
						dataRp = null;
						time1 = System.currentTimeMillis();
						if(QUEUETIMER)
							System.out.println("Queued framesets:"+(sequenceNumber-lastSequenceNumber)+" file index:"+files);
						lastSequenceNumber = sequenceNumber;
					}
				//if( DEBUG ) {
				//	System.out.println("New left/right images "+img.getWidth()+","+img.getHeight()+" size:"+bufferL.length/*ib.limit()*/);
				//}
			} catch (IOException e1) {
				System.out.println("Could not convert image payload due to:"+e1.getMessage());
				return;
			}
			++sequenceNumber; // we want to inc seq regardless to see how many we drop	
		}
	  });
		
	} // onstart
	
	final static class TreeComp implements Comparator<Long> {
		  public int compare(Long csk1, Long csk2) {
			  return csk1.compareTo(csk2);
		  }
	}
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
					  SynchronizedFixedThreadPoolManager.getInstance().init(numThreads, execLimit, "BUILDOCTREE");
					  for(int syStart = 0; syStart < execLimit; syStart++) {
						SynchronizedFixedThreadPoolManager.getInstance(numThreads, execLimit, "BUILDOCTREE").spin(new Runnable() {
						  @Override
						  public void run() {
							// Since we run a continuous loop inside run we have to have a way
							// to synchronize everything at the beginning of a new image, then
							// await completion of all processing threads and signal a reset to
							// resume processing at the start of another new image, hence the two barrier
							// synchronization latches
								imagesToOctrees(dataL, dataR, imageLx, imageRx, yStart.getAndIncrement(), camWidth, camHeight, nodel, noder/*hlL, hlR*/);
						  } // run
					    },"BUILDOCTREE"); // spin
					  } // for syStart
					  SynchronizedFixedThreadPoolManager.getInstance().waitForGroupToFinish("BUILDOCTREE");
					  if( TIMER )
						  System.out.println("Process time one="+(System.currentTimeMillis()-etime));
					  latchOut.await();
					  //
					  // next parallel processing step, if any
					  //
					  //final Comparator<octree_t> yComp = new Comparator<octree_t>() {  // descending centroid   
					  	// @Override         
						// public int compare(octree_t jc1, octree_t jc2) {             
						//	      return (jc2.getCentroid().y < jc1.getCentroid().y ? -1 :                     
						//	              (jc2.getCentroid().y == jc1.getCentroid().y ? 0 : 1));           
						// }     
					  //};
					  //final Comparator<octree_t> yComp = new Comparator<octree_t>() {  // ascending middle      
						//  @Override         
						//  public int compare(octree_t jc1, octree_t jc2) {             
						//	      return (jc1.getMiddle().y < jc2.getMiddle().y ? -1 :                     
						//	              (jc1.getMiddle().y == jc2.getMiddle().y ? 0 : 1));           
						//  }     
					  //};
					  latch2.await();
					  etime = System.currentTimeMillis();
					  yStart.set(0);
					  final ArrayList<octree_t> tnodel = nodel.get_nodes();
					  ArrayList<octree_t> tnoder = noder.get_nodes();
					  //Collections.sort(tnodel, yComp);
					  //Collections.sort(tnoder, yComp);
					  //final List<octree_t> nodelA = Collections.synchronizedList(tnodel);
					  //final List<octree_t> noderA = Collections.synchronizedList(tnoder);
					  indexDepth = Collections.synchronizedList(new ArrayList<envInterface>());
					  indexUnproc = Collections.synchronizedList(new ArrayList<envInterface>());
					  leftYRange = Collections.synchronizedList(new ArrayList<int[]>());
					  final RadixTree<Integer, octree_t> txr = new RadixTree<Integer, octree_t>();
					  //final TreeMap<Long, octree_t> txr = new TreeMap<Long, octree_t>(new TreeComp());
					  //final SortedMap<Long, octree_t> txr = Collections.synchronizedSortedMap(new TreeMap<Long, octree_t>(new TreeComp()));
					  genDCT2(tnoder, txr);
					  //System.out.println("LEFT ChromoSpatial key table size = "+txl.size()+" with "+vertl+" vertical");
					  //System.out.println("RIGHT ChromoSpatial key table size = "+txr.size()+" with "+vertr+" vertical");
					  //float[] tx = genDCT3(txr.keys(), nodelA.size());
					  // cast to int to compress list, fractional tolerances ignored
					  //int y = (int) nodelA.get(0).getMiddle().y;
					  int iPosStart = 1;
					  int iPosEnd = 1;
					  //for(int i = 0 ; i < nodelA.size(); i++) {
						  //System.out.println("centroid="+(int)nodelA.get(i).getCentroid().y);
						   //if( y != (int)nodelA.get(i).getMiddle().y) {
							//   iPosEnd = i;
							//   leftYRange.add(new int[]{iPosStart, iPosEnd});
							//   iPosStart = i;
							//   y = (int) nodelA.get(i).getMiddle().y;
						   //}
					  //}
					  //iPosEnd = nodelA.size();
					  int incr = tnodel.size()/16;
					  for(int i = 0; i < incr; i++){
						  iPosEnd +=16;
						  leftYRange.add(new int[]{iPosStart, iPosEnd-1});
						  iPosStart = iPosEnd;
					  }
					  iPosEnd = tnodel.size();
					  leftYRange.add(new int[]{iPosStart, iPosEnd});
					  final int nSize = leftYRange.size();
					  final int nSizeT = Math.min(leftYRange.size(), 16);
					  //System.out.println("NSize="+nSize+" NSizeT="+nSizeT);
					  //for(int i = 0; i < leftYRange.size(); i++) {
						//  System.out.println(leftYRange.get(i)[0]+" "+leftYRange.get(i)[1]);
					  //}
					  SynchronizedFixedThreadPoolManager.getInstance().init(nSizeT, nSize, "MATCHREGION");
					  for(int syStart = 0; syStart < nSize; syStart++) {
							SynchronizedFixedThreadPoolManager.getInstance(nSizeT, nSize, "MATCHREGION").spin(new Runnable() {
								@Override
								public void run() {
									// set the left nodes with depth
									matchRegionsAssignDepth(yStart.getAndIncrement(), leftYRange, camWidth, camHeight, tnodel, txr, indexDepth, indexUnproc);
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
					} catch (BrokenBarrierException | InterruptedException e) { System.out.println("<<BARRIER BREAK>> "+files);}
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
					double meanRMS = 0;
					double varianceRMS = 0;
					double sigmaRMS = 0;
					System.out.println("Image queue..");
					/**
					 * Main processing loop, extract images from queue, notify worker threads, then display disparity
					 */
			        while(true) {
			        	if( queueI.isEmpty() ) {
							try {
								Thread.sleep(1);
							} catch (InterruptedException e) {}
			        	} else {
			        		try {
			        			// If we are waiting at either cyclic barrier, the reset will cause it to
			        			// to return to top level barrier
			        			// the latches represent each 'step', or group of parallel processing tasks
			        			// use as many steps as needed, unused latches ignored
			        			Object[] o = queueI.takeFirst();
								imageLx = (BufferedImage) o[0];
				        		imageRx = (BufferedImage) o[1];
				        		dataL = (int[]) o[2];
				        		dataR = (int[]) o[3];
			        			latchOut4.reset();
			        			latch4.reset();
			        			latchOut3.reset();
			        			latch3.reset();
			        			latchOut2.reset();
			        			latch2.reset();
			        			latchOut.reset();
			        			latch.reset();
			        			//
				        	    imageL = imageLx;
				        	    imageR = imageRx;	
			        			nodel = new octree_t();
			        			octree_t.buildStart(nodel);
			        			noder = new octree_t();
			        			octree_t.buildStart(noder);
			        		   	// write source images
			        			++files;
			        			if( WRITEFILES || WRITEJPEGS) {
			        				synchronized(imageL) {
			        					writeFile("sourceL", imageL, files);
			        				}
			        				synchronized(imageR) {
			        					writeFile("sourceR", imageR, files);
			        				}
			        			}
			        			// metrics
			        			metric1 = metric2 = metric3 = metric4 = metric5 = 0;
			        	     	// first step end multi thread barrier synch
			        			latch.await();
			        	     	latchOut.await();
			        	     	synchronized(nodel) {
			        	     		octree_t.buildEnd(nodel);
			        	     		hough_settings.s_level = 5;
			        	     		hough_settings.max_distance2plane = 1;
			        	     		// min_isotropy is how much PCA eigenvalue variance2/variance3 tolerance allowed 
			        	     		// in order to subdivide octree at each level
			        	     		hough_settings.min_isotropy = .01;
			        	     		nodel.subdivide();
			        	     		//writeFile(nodel,"/roscoeL"+(++frames));
			        	     		if( WRITEFILES || WRITEPLANARS)
			        	     			writer_file.writePerp(nodel, "planarsL"+files);
			        	     	}
			        	     	synchronized(noder) {
			        	     		octree_t.buildEnd(noder);
			        	     		noder.subdivide();
			        	     		if(WRITEFILES || WRITEMODEL) {
			        	     			writeFile(noder,"/roscoeR"+files);
			        	     		}
			        	     		if( WRITEFILES || WRITEPLANARS) {
			        	     			writer_file.writePerp(noder, "planarsR"+files);
			        	     		}
			        	     	}
			        	     	// second step end multi thread barrier synch
			        	     	latch2.await();
			        	     	latchOut2.await();
			        	     	// write unmatched minimal envelopes
			        	     	int empty = 0;
			        	     	int wasvotes = 0;
		        	     		synchronized(indexUnproc) {
		        	     			for(envInterface e: indexUnproc) {
		        	     				if(e.getDepth() == 0)
		        	     					++empty;
		        	     				else
		        	     					++wasvotes;
		        	     			}
		        	     		}
		        	     		System.out.println("Run had "+empty+" sets and "+wasvotes+" instances of all elementes voted");
			        	     	if( WRITEFILES || WRITEUNCORR) {
			        	     		synchronized(indexUnproc) {
			        	     			writeFile(indexUnproc, "/lvl7uncorrL"+files);
			        	     		}
			        	     	}
			        	     	if( WRITEFILES || WRITECORR) {
			        	     		synchronized(indexDepth) {
			        	     			writeFile(indexDepth, "/lvl7corrL"+files);
			        	     		}
			        	     	}
								// calc mean
			        	     	
			        	      	double[] a = null; // DCT elements
								mean = variance = 0;
								if( indexDepth.size() > 0) {
									synchronized(indexDepth) {
									  for(envInterface ind : indexDepth) {
										  mean += ind.getDepth();
									  }
									  mean /= (double)indexDepth.size();
									  for(envInterface ind : indexDepth) {
										  variance += Math.pow((mean - ind.getDepth()),2);
									  }
									  variance /= (double)indexDepth.size();
									  int isize = indexDepth.size();
									  a = genDCT1(indexDepth, isize);
									}
									sigma = Math.sqrt(variance); 
									if(prevDCT != null) {
									  double rms = IOUtils.computeRMSE(a, prevDCT, Math.min(a.length, prevDCT.length));
									  meanRMS += rms;
									  meanRMS /= 2.0;
									  varianceRMS += Math.pow((meanRMS - rms),2);
									  varianceRMS /= 2.0;
									  sigmaRMS = Math.sqrt(varianceRMS);
									  System.out.println("DCT RMS Mean="+meanRMS+" variance="+varianceRMS+" standard deviation="+sigmaRMS);
			        	     		  System.out.println("RMS err="+rms+" file index:"+files);
			        				  // if we are more than n standard deviation from the overall mean, do something
			    					  if( rms > (meanRMS+(sigmaRMS*DCTRMSSigma)) ) { //sigmaRMS*2 is 2 standard deviations
			        	     			System.out.println("<<<< IMAGE SHIFT ALERT!!!! >>>> rms="+rms+" err="+(meanRMS+(sigmaRMS*1.3))+" file index:"+files);
			        	     			if( WRITERMS )
			        	     				synchronized(nodel) {
			        	     					writeFile(nodel,"/RMS"+(int)rms+"."+files);
			        	     				}
			    					  }
									}
									prevDCT = a;
								}
								System.out.println("Mean depth="+mean+" variance="+variance+" standard deviation="+sigma);		
			        	     	System.out.println("Uncorrelated regions="+indexUnproc.size()+" correlated="+indexDepth.size()+", "+(((float)indexUnproc.size()/(float)(indexUnproc.size()+indexDepth.size()))*100.0)+"% uncorrelated");
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
			        	     		Iterator<envInterface> it = zeroEnc.iterator();
			        	     		//for(envInterface e : zeroEnc) {
			        	     		while(it.hasNext()) {
			        	     			envInterface e = (envInterface) it.next();
			        	     			if(e.getDepth() != 0) {
			        	     				maxEnv.add(e);
			        	     				it.remove();
			        	     			}
			        	     		}
			        	     		// write the display cloud with maximal envelopes
			        	     		if(WRITEFILES || WRITEZEROENC)
			        	     				writeFile(zeroEnc, "/lvl5zeroenvL"+files);
			        	     		System.out.println("Final zero enclosing maximal envelopes="+zeroEnc.size());
			        	     		System.out.println("Metrics="+metric1+", "+metric2+", "+metric3+", "+metric4+" ,"+metric5);
			        	     	}
			        	     	// end of parallel processing
			        	     	synchronized(nodel) {
			        	     		// at this point processing of image is essentially complete. If ASSIGNPOINTS is true
			        	     		// the z coords in the point cloud are populated, otherwise we have a list of maximal
			        	     		// planar regions with depth.
			        	     		if(WRITEFILES || WRITEMODEL)
			        	     			writeFile(nodel,"/roscoeL"+files);
			        	     		//
			        	     		// write the remaining unprocessed envelopes from minimal
			        	     		if(WRITEFILES || WRITEENCZERO)
			        	     			if(!indexDepth.isEmpty())
			        	     				synchronized(indexDepth) {
			        	     					writeFile(indexDepth,"/lvl7unencL"+files);
			        	     				}
			        	     		synchronized(maxEnv) {
			        	     			if(WRITEFILES || WRITEPLANES)
			        	     				writeFile(maxEnv, "/lvl5MaxEnv"+files);
			        	     			if(WRITEFILES || WRITEGRID)
			        	     				genNav2(maxEnv);
			        	     			//a = genDCT(maxEnv);
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
	 * Translate 2 PixelsToModel integer linear arrays of edge data and their corresponding RGB images
	 * into 2 octrees. Intended to be 1 scan line in multithreaded parallel thread group.
	 * @param imageL Left image result of PixelsToModel
	 * @param imageR Right image from PixelsToModel
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
 			System.arraycopy(Arrays.copyOfRange(imageL, yStart*width, (yStart+1)*width), 0, imgsrcL, 0, width);
 		}
		synchronized(imageLx2) {
			imageLx2.getRGB(0, yStart, width, 1, imgsrcLx, 0, width);
		}
		synchronized(imageR) {
 			System.arraycopy(Arrays.copyOfRange(imageR, yStart*width, (yStart+1)*width), 0, imgsrcR, 0, width);
 		}
		synchronized(imageRx2) {
			imageRx2.getRGB(0, yStart, width, 1, imgsrcRx, 0, width);
		}
	  	for(int xsrc = 0; xsrc < width; xsrc++) {
	  		// If the left image pixel which is the target to receive the depth value is not edge, continue
			//if(imgsrcL[xsrc] == 0)
			//		continue;
			double ks = xsrc - (width/2);
			double ms = yStart - (height/2);
			double os = imgsrcL[xsrc]; // from PixelsToModel if no depth in model set to 1

			synchronized(nodel) {
				octree_t.build(nodel, (double)ks, (double)ms, os, 
					((imgsrcLx[xsrc] & 0x00FF0000) >> 16 ), 
					((imgsrcLx[xsrc] & 0x0000FF00) >> 8), 
					((imgsrcLx[xsrc] & 0x000000FF) ));
			}
		} //++xsrc  move to next x subject in left image at this scanline y
  		for(int xsrc = 0; xsrc < width; xsrc++) {
			// If the left image pixel which is the target to receive the depth value is not valid, continue
			//if(imgsrcR[xsrc] == 0)
			//	continue;
			double ks = xsrc - (width/2);
			double ms = yStart - (height/2);
			double os = imgsrcR[xsrc]; // mean color depth or 1

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
	 * that has the minimum difference in eigenvector dot. Where they are the same compare eigenvalue and take the
	 * least difference. Finally, count the points in the region and find closest. In the case of vertical
	 * lines with multiple matches, reject the match, if one match to a vertical line, then use that.
	 * Take the distance between centroids as disparity.
	 * @param yStart position leftYRange containing from/to positions in left node list for this thread.
	 * @param leftYRange2 list containing from/to of common Y centroid elements in sorted left node list
	 * @param width width of image
	 * @param height height of image
	 * @param nodelA list of left octree nodes at smallest level, sorted by centroid y
	 * @param txr list of right octree nodes at smallest level, sorted by centroid y
	 * @param indexDepth2 resulting array filled with envInterface instances of matched regions
	 * @param indexunproc2 resulting array of envInterface filled with unmatched minimal regions
	 */
	private void matchRegionsAssignDepth(int yStart, 
										List<int[]> leftYRange2, 
										int width, int height, 
										List<octree_t> txl, RadixTree<Integer, octree_t> txr,
										List<envInterface> indexDepth2, List<envInterface> indexUnproc2) {
		long etime = System.currentTimeMillis();
		octree_t inode;
		octree_t oscore = null;
		//List<octree_t> leftNodes;
		//List<octree_t> rightNodes;
		// get all nodes along this Y axis, if any
		// we are just trying to establish a range to sublist
		//synchronized(nodelA) {
		int[] irange = leftYRange2.get(yStart);
			//leftNodes = nodelA.subList(irange[0], irange[1]);
		//}
		// get the right node that matches left as best as possible
		for(int i = irange[0]; i < irange[1]; i++) {
			if(i >= txl.size()) {
				return;
			}
			boolean found = false;
			inode = txl.get(i);
			if( inode.getVariance1() == 0 ) {
				indexUnproc2.add(new IndexDepth(inode, 0));
				continue;
			}
			double sum = Double.MAX_VALUE;
			int nscore = 0;
			short[] vals = genChromoSpatialKeys2(inode);
			//int options = (vals[2] << 20) |(vals[3] << 10) | vals[4];
			//SortedMap<Integer, octree_t> noderD = txr.subMap(vals[0], vals[1], (short)0xFFFFFFC0, (short)0x3F); // last args is bit mask for search low and high, and and or
			SortedMap<Integer, octree_t> noderD = txr.subMap(vals[0], vals[1], (short)0xFFFFFF00, (short)0xFF); // last args is bit mask for search low and high, and and or 2 bits at a time for both tolerances
			if(noderD.size() == 1) {
				nscore = 1;
				oscore = noderD.values().iterator().next();
			} else {
				for(octree_t jnode : noderD.values()) {
					//
					synchronized(jnode) {
						if( jnode.getVotes() == 1 ) {
							continue;
						}
					}
					//Vector4d cross = inode.getNormal1().multiplyVectorial(jnode.getNormal1());
					//double dot = inode.getNormal1().and(jnode.getNormal1()); // dot
					//double crossMag = Math.sqrt(cross.x*cross.x + cross.y*cross.y + cross.z*cross.z);
					//double angle = Math.atan2(crossMag, dot);
					//double vscore1 = Math.abs(inode.getVariance1()-jnode.getVariance1());
					//double vscore2 = Math.abs(inode.getVariance2()-jnode.getVariance2());
					//double vscore3 = Math.abs(inode.getVariance3()-jnode.getVariance3());
					//if(angle > aTolerance /*|| vscore1 > bTolerance || vscore2 > cTolerance || vscore3 > dTolerance*/) {
					//	continue;
					//}
					++nscore;
					found = true;
					//System.out.println("ANGLE:="+angle+" "+vscore1+" "+vscore2+" "+vscore3+"\r\n"+inode+"\r\n"+jnode);
					// we may have to SAD the coplanar attributes
					//double sadf = genSAD(inode, jnode, Math.min(inode.getIndexes().size(), jnode.getIndexes().size()));
					double sadf = genDCA(inode, jnode);
					if( sadf < sum ) {
						sum = sadf;
						oscore = jnode;
					}
				}
				// either set was empty or only voted upon nodes were encountered
				if(!found) {
					if(DEBUGTEST2)
						System.out.println("matchRegionsAssignDepth left node "+inode+" no right image nodes in tolerance ***** "+Thread.currentThread().getName()+" ***" );
					if( noderD.isEmpty())
						indexUnproc2.add(new IndexDepth(inode, 0));
					else
						indexUnproc2.add(new IndexDepth(inode, .1));
					continue;
				}
				synchronized(oscore) {
					if( nscore == 1 && oscore.getVotes() == 1 ) {
						System.out.println("Only rerieved node has already been matched! "+oscore);
					} else
						oscore.setVotes(1);
				}
			}
			octree_t jnode = oscore;
			double xDiff = Math.abs(inode.getCentroid().x-jnode.getCentroid().x);
			double yDiff =  Math.abs(inode.getCentroid().y-jnode.getCentroid().y);
			//calc the disparity and insert into collection
			//we will call disparity the distance to centroid of right
			double pix = Bf/Math.hypot(xDiff, yDiff);
			Vector4d cross = inode.getNormal1().multiplyVectorial(jnode.getNormal1());
			double dot = inode.getNormal1().and(jnode.getNormal1()); // dot
			double crossMag = Math.sqrt(cross.x*cross.x + cross.y*cross.y + cross.z*cross.z);
			double angle = Math.atan2(crossMag, dot);
			System.out.println("Found "+nscore+" Angle="+Math.toDegrees(angle)+" disp="+pix+" ***** "+Thread.currentThread().getName()+" ***");
			//if( pix >=maxHorizontalSep) {
			//	if(DEBUGTEST2)
			//		System.out.println("matchRegionsAssignDepth left node "+inode+" right node "+jnode+" PIX TRUNCATED FROM:"+pix+" ***** "+Thread.currentThread().getName()+" *** ");
			//	pix = maxHorizontalSep;
			//}
			// insert it into the synchronized list of maximal envelopes
			if( pairCount < WRITEMATCHEDPAIRS) {
				IndexDepth d1 = new IndexDepth(inode, pix);
				IndexDepth d2 = new IndexDepth(jnode, pix);
				writeTestPerp(d1, d2,"match"+(pairCount++));
				indexDepth2.add(d1);
			} else
				indexDepth2.add(new IndexDepth(inode, pix));
		}
	
		// We have a set of left nodes with a common Y value, now extract all right nodes with a Y value in tolerance of
		// our left node set upon which this thread is operating. 
		// Other similar left node sets exist upon which other threads are operating.
		/*
		int rpos1 = 0;
		int rpos2 = 0;
		synchronized(txr) {		
			octree_t lnode = leftNodes.get(0);
			loop0:
			for(int j = 0; j < txr.size(); j++) {
				double yDiff =  Math.abs(lnode.getCentroid().y-txr.get(j).getCentroid().y);
				if( yDiff <= yTolerance ) {
					rpos1 = j;
					found = true;
					for(int k = j; k < txr.size(); k++) {
						yDiff =  Math.abs(lnode.getCentroid().y-txr.get(k).getCentroid().y);
						if(yDiff > yTolerance) {
							rpos2 = k;
							break loop0;
						}
					}
					rpos2 = txr.size();
					break;
				}
			}
			if(!found) {
				for( octree_t inode : leftNodes) {
					if(DEBUGTEST2)
						System.out.println("matchRegionsAssignDepth left node "+inode+" no right image nodes in tolerance ***** "+Thread.currentThread().getName()+" ***" );
					indexUnproc2.add(new IndexDepth(inode, 0));
				}
				return;
			}
			rightNodes = txr.subList(rpos1, rpos2);
		}
		*/

		/*for( octree_t inode : leftNodes) {
			double sum = Double.MAX_VALUE;
			double csum = Double.MAX_VALUE;
			synchronized(inode) {
				// dont try to match a horizontal or vertical line, no confidence factor
				if( inode.getVariance1() == 0.0 
					//(inode.getNormal1().x == 0 && inode.getNormal1().y == 0) ||
					//(inode.getNormal1().x == 0 && inode.getNormal1().y == 1) ||
					//(inode.getNormal1().x == 1 && inode.getNormal1().y == 0)) {
					if(DEBUGTEST2)
						System.out.println("matchRegionsAssignDepth rejection left node "+inode+" horizontal or vertical ***** "+Thread.currentThread().getName()+" ***");
					indexUnproc2.add(new IndexDepth(inode, 0));
					continue; // next left node
				}
				int isize = inode.getIndexes().size();
				//int zsizeMax = (int)inode.getCentroid().z + (int)Math.ceil(inode.getSize()/2) + zTolerance;
				//int zsizeMin = (int)inode.getCentroid().z - (int)Math.ceil(inode.getSize()/2) - zTolerance;
				// check all right nodes against the ones on this scan line
				found = false;
				int right = 0;
				octree_t oscore = null;
				//
				for(octree_t jnode: rightNodes) {
					synchronized(jnode) {
						int jzsizeMax = (int)jnode.getCentroid().z + (int)Math.ceil(jnode.getSize()/2);
						int jzsizeMin = (int)jnode.getCentroid().z - (int)Math.ceil(jnode.getSize()/2);
						if( jnode.getVotes() != 0 || 
							(jzsizeMin < zsizeMin && jzsizeMax < zsizeMin) || (jzsizeMin > zsizeMin && jzsizeMax > zsizeMax))
							continue;
					}
					// found in range, acos of dot product
					double cscore = Math.acos(inode.getNormal1().and(jnode.getNormal1()));
					//double tcscore = Math.abs(inode.getVariance2()-jnode.getVariance2());
					//double ecscore = Math.abs(inode.getVariance3()-jnode.getVariance3());
					int iscore = (Math.abs(isize-jnode.getIndexes().size())*100)/isize;
					if(cscore <= aTolerance && iscore <= cTolerance) {
						found = true;
						double osum = 0;
						int nsize = jnode.getIndexes().size();
						if(isize < nsize)
							nsize = isize;
						osum = genSAD(inode, jnode, nsize);
						//osum = genDCT2(inode, jnode, nsize);
						if(osum < sum) {
							oscore = jnode;
							sum = osum;
							++right;
						}
					} else {
					    ++nscore; // off plane, beyond tolerance
						if(DEBUGTEST2) // this creates a massive amout of output quickly
							System.out.println("matchRegionsAssignDepth rejection-\r\nleft node :"+inode+"\r\nright node:"+jnode+"\r\nangle dot="+cscore+" point diff="+iscore+" right="+right+" wrong="+nscore+" ***** "+Thread.currentThread().getName()+" ***");
						if(cscore > aTolerance) 
							++metric1;
						else
							//if(tcscore > bTolerance)
							//	++metric2;
							//else
							//	if(ecscore > b2Tolerance)
							//		++metric3;
							//	else
									if(iscore > cTolerance)
										++metric4;
					}
			    } // right node loop
				metric5 += right;
				
				// get left node key
				long lnode[] = genChromoSpatialKeys(inode);
				// see if match
				oscore = txr.get(lnode[1]);
				// cant get straightup match, try range
				if( oscore == null) {
					long[] srange = genChromoSpatialKeyRange(lnode[1]);
					SortedMap<Long, octree_t> smap = txr.subMap(srange[0], srange[1]);
					//Entry<Long, octree_t> fscore = txr.floorEntry(lnode[1]);
					//Entry<Long, octree_t> cscore = txr.ceilingEntry(lnode[1]);		
					if(smap == null || smap.isEmpty()) {					
							if(DEBUGTEST2) 
								System.out.println("matchRegionsAssignDepth rejection left node"+inode+" no matching right node out of "+nscore+" ***** "+Thread.currentThread().getName()+" ***");
							indexUnproc2.add(new IndexDepth(inode, 0));
							continue;
					}
					Collection<octree_t> svals = smap.values();
					//System.out.println("Range subset Got "+svals.size());
					Iterator<octree_t> siter = svals.iterator();
					synchronized(txr) {
					while(siter.hasNext()) {
						octree_t jnode = siter.next();
						synchronized(jnode) {
							if( jnode.getVotes() == 1)
								continue;
						}
						double cscore = Math.acos(inode.getNormal1().and(jnode.getNormal1()));
						double vscore1 = Math.abs(inode.getVariance1()-jnode.getVariance1());
						double vscore2 = Math.abs(inode.getVariance2()-jnode.getVariance2());
						double vscore3 = Math.abs(inode.getVariance3()-jnode.getVariance3());
						if(cscore > aTolerance || vscore1 > bTolerance || vscore2 > bTolerance || vscore3 > bTolerance) {
							continue;
						}
						// we may have to SAD them
						// yes, SAD
						double sadf = genSAD(inode, jnode, Math.min(inode.getIndexes().size(), jnode.getIndexes().size()));
						if( sadf < sum ) {
							sum = sadf;
							oscore = jnode;
						}
					}
					}
				} else
					System.out.println("DIRECT MATCH:"+inode+" TO \r\n"+oscore+" for key:"+lnode[1]);
				// set right node as matched
				synchronized(oscore) {
					oscore.setVotes(1);
				}
				if(DEBUGTEST2 && right > 1)
					System.out.println("Found "+right+" right nodes for left node "+inode+" out of tolerance nodes="+nscore+" ***** "+Thread.currentThread().getName()+" ***");
				double xDiff = Math.abs(inode.getCentroid().x-oscore.getCentroid().x);
				double yDiff =  Math.abs(inode.getCentroid().y-oscore.getCentroid().y);
				//calc the disparity and insert into collection
				//we will call disparity the distance to centroid of right
				double pix = Bf/Math.hypot(xDiff, yDiff);	
				if( pix >=maxHorizontalSep) {
					if(DEBUGTEST2)
						System.out.println("matchRegionsAssignDepth left node "+inode+" right node "+oscore+" PIX TRUNCATED FROM:"+pix+" ***** "+Thread.currentThread().getName()+" *** ");
					pix = maxHorizontalSep;
				}
				// insert it into the synchronized list of maximal envelopes
				if( pairCount < WRITEMATCHEDPAIRS) {
					IndexDepth d1 = new IndexDepth(inode, pix);
					IndexDepth d2 = new IndexDepth(oscore, pix);
					writeTestPerp(d1, d2,"match"+(pairCount++));
					indexDepth2.add(d1);
				} else
					indexDepth2.add(new IndexDepth(inode, pix));
				//
				if(DEBUGTEST2 && ASSIGNPOINTS)
					System.out.println("matchRegionsAssignDepth left node "+inode+" right node "+oscore+" should set set "+isize+" points to "+pix+" ***** "+Thread.currentThread().getName()+" *** ");
			} // inode synch
		} // left octree nodes
		*/

		if( SAMPLERATE )
			System.out.println("matchRegionAssignDepth ***** "+Thread.currentThread().getName()+" *** "+(System.currentTimeMillis()-etime)+" ms.***");
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
	 * @param maxEnv maximal envelopes to which a depth was assigned, a list filled by this method
	 * @param zeroEnc zero enclosing maximal envelopes that did not go to maxEnv, a list filled by this method
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
			}
			// move the former zero encloser to maximal nodes with mean of all the maximals that enclose it/it encloses
			senv.setDepth(smean);
			return;
		}
		//} synchronized env
		if(DEBUGTEST4)
			System.out.println("findZeroEnclosedSetPointDepth node "+yStart+"="+inode+" found no valid alternate envelope ***** "+Thread.currentThread().getName()+" *** ");	
	}
	
	//------------------------------------------
	// support methods
	//------------------------------------------
	private static double genDCA(octree_t inode, octree_t jnode) {
		double[] weights = new double[]{10.0, 1.5, .2, .01};
		if( inode.getParent() == null ) {
			System.out.println("No parent for "+inode);
			return Double.MAX_VALUE;
		}
		if( jnode.getParent() == null) {
			System.out.println("No parent for "+jnode);
			return Double.MAX_VALUE;
		}
		octree_t[] inodes = inode.getParent().getChildren();
		octree_t[] jnodes = jnode.getParent().getChildren();
		double res = 0;
		for(int i = 0; i < 8; i++) {
			if( inodes[i].getNormal1() == null ) {
				//System.out.println("No child normal "+i+" for inode "+inode);
				continue;
			}
			if( jnodes[i].getNormal1() == null) {
				//System.out.println("No child normal "+i+" for jnode "+jnode);
				continue;
			}
			Vector4d cross = inodes[i].getNormal1().multiplyVectorial(jnodes[i].getNormal1());
			double dot = inodes[i].getNormal1().and(jnodes[i].getNormal1()); // dot
			double crossMag = Math.sqrt(cross.x*cross.x + cross.y*cross.y + cross.z*cross.z);
			double angle = Math.atan2(crossMag, dot)*weights[0];
			double vscore1 = Math.abs(inodes[i].getVariance1()-jnodes[i].getVariance1())*weights[1];
			double vscore2 = Math.abs(inodes[i].getVariance2()-jnodes[i].getVariance2())*weights[2];
			double vscore3 = Math.abs(inodes[i].getVariance3()-jnodes[i].getVariance3())*weights[3];
			res += (angle + vscore1 + vscore2 + vscore3);
		}
		return res;
	}
	private static double[] genDCT1(List<envInterface> nodes, int size) {
		double[] coeffs = genCoeffs1(nodes, size);
		DoubleDCT_1D fdct1d = new DoubleDCT_1D(size);
		fdct1d.forward(coeffs, false);
		return coeffs;
	}
	private static double[] genCoeffs1(List<envInterface> nodes, int size) {
		   double[] coeffs = new double[size];
		   for(int i = 0; i < size; i++) {
			   envInterface nodex = nodes.get(i);
			   octree_t node = ((AbstractDepth)nodex).node;
			   long[] vals = genChromoSpatialKeys(node);
			   // our rho for 2 is eigenvalue
			   // form the vector from normal2 and its magnitude (eigenvalue) toward centroid to origin 0,0 center
			   //long val = ((short)degPhi2)<<48 | ((short)node.getVariance2())<<32 | ((short)degPhi)<<16 | (short)sph1[2];
			   //if(DEBUG)
			   //   System.out.println("val "+i+"="+val+" (float)val="+(float)val+" -- "+(short)degPhi2+" "+(short)node.getVariance2()+" "+(short)degPhi+" "+(short)sph1[2]);
			   //coeffs[i][0] = (float)vals[0];
			   //coeffs[i][1] = (float)vals[1];	
			   coeffs[i] = (double)vals[1];
		   }
		   return coeffs;
	}
	private static void genDCT2(List<octree_t> nodes, RadixTree<Integer, octree_t> txr) {
		int dup = 0;
		int vert = 0;
		int i = 0;
		for(octree_t node: nodes) {
			if(node.getVariance1() != 0) {
				short[] vals = genChromoSpatialKeys2(node);
				//int options = (vals[2] << 20) |(vals[3] << 10) | vals[4];
				octree_t ot = txr.put(vals[0], vals[1], node);
				if( ot != null )
					++dup;
				else
					++i;
			} else
				 ++vert;
		}
		System.out.println("Built "+i+" element radix tree from "+nodes.size()+" nodes with "+vert+" vertical and "+dup+" duplicated");
	}
	private static float[] genDCT3(Enumeration<Float> nodes, int size) {
		float[] coeffs = genCoeffs3(nodes, size);
		FloatDCT_1D fdct1d = new FloatDCT_1D(size);
		fdct1d.forward(coeffs, false);
		return coeffs;
	}
	private static float[] genCoeffs3(Enumeration<Float> nodes, int size) {
		   float[] coeffs = new float[size];
		   for(int i = 0; i < size; i++) {
			   coeffs[i] = nodes.nextElement();
			   // our rho for 2 is eigenvalue
			   // form the vector from normal2 and its magnitude (eigenvalue) toward centroid to origin 0,0 center
			   //long val = ((short)degPhi2)<<48 | ((short)node.getVariance2())<<32 | ((short)degPhi)<<16 | (short)sph1[2];
			   //if(DEBUG)
			   //   System.out.println("val "+i+"="+val+" (float)val="+(float)val+" -- "+(short)degPhi2+" "+(short)node.getVariance2()+" "+(short)degPhi+" "+(short)sph1[2]);
			  // coeffs[i][0] = (float)vals[0];
			  //
		   }
		   return coeffs;
	}
	/**
	 * val[0] center to planar, val[1] centroid to normal
	 * @param node
	 * @return
	 */
	private static long[] genChromoSpatialKeys(octree_t node) {
		   double[] sph1 = octree_t.cartesian_to_spherical(node.getCentroid()); //1=theta,2=phi,3=rho
		   double degTheta = Math.toDegrees(sph1[0]);
		   if( degTheta < 0.0) {
			    degTheta += 360.0;
		   }
		   double degPhi = Math.toDegrees(sph1[1]);
		   if( degPhi < 0.0) {
			    degPhi += 360.0;
		   }
		   // deg*100 = 0-36099 = 1000110100000011b
		   long val1 = (((long)(degTheta*100.0d) & 0xFFFF) << 48) | (((long)(degPhi*100.0d) & 0xFFFF) << 32) | ((long)(sph1[2]*100.0d) );
		   // axis of middle variance, perp to normal
		   double[] sph2 = octree_t.cartesian_to_spherical(node.getNormal1());
		   double degTheta2 = Math.toDegrees(sph2[0]);
		   if( degTheta2 < 0.0) {
			    degTheta2 += 360.0;
		   }
		   double degPhi2 = Math.toDegrees(sph2[1]);
		   if( degPhi2 < 0.0) {
			    degPhi2 += 360.0;
		   }
		   // 0x3FF 10 bits 1023 or 102.3 max variance
		   long val2 = (((long)(degTheta2*100.0d) & 0xFFFF) << 48) | (((long)(degPhi2*100.0d) & 0xFFFF) << 32) | 
				   (((long)(node.getVariance1()*10.0d) & 0x3FF) << 20) | (((long)(node.getVariance2()*10.0d) & 0x3FF) << 10) | (((long)(node.getVariance3()*10.0d) & 0x3FF)); 

		   return new long[]{val1, val2};
	}
	/**
	 * 
	 * @param node
	 * @return 5 shorts of node value normal1: theta, phi, variance1, variance2, variance3
	 */
	private static short[] genChromoSpatialKeys2(octree_t node) {
		   double[] sph1 = octree_t.cartesian_to_spherical(node.getNormal1()); //1=theta,2=phi,3=rho
		   double degTheta = Math.toDegrees(sph1[0]);
		   if( degTheta < 0.0) {
			    degTheta += 360.0;
		   }
		   double degPhi = Math.toDegrees(sph1[1]);
		   if( degPhi < 0.0) {
			    degPhi += 360.0;
		   }
		   // deg*100 = 0-36099 = 1000110100000011b
		   short val1 = (short)(((int)(degTheta*10.0d)) & 0x7FFF);
		   short val2 = (short)(((int)(degPhi*10.0d)) & 0x7FFF);
		   if( val1 < 0 || val2 < 0 )
			   System.out.println("OVERFLOW CHROMOSPATIAL KEY VALUE!! "+val1+" "+val2);
		   short val3 = (short)(((int)(node.getVariance1()*10.0d)) & 0x3FF);
		   short val4 = (short)(((int)(node.getVariance2()*10.0d)) & 0x3FF);
		   short val5 = (short)(((int)(node.getVariance3()*10.0d)) & 0x3FF);
		   // 0x3FF 10 bits 1023 or 102.3 max variance
		   //long val2 = (((long)(degTheta2*100.0d) & 0xFFFF) << 48) | (((long)(degPhi2*100.0d) & 0xFFFF) << 32) | 
			//	   (((long)(node.getVariance1()*10.0d) & 0x3FF) << 20) | (((long)(node.getVariance2()*10.0d) & 0x3FF) << 10) | (((long)(node.getVariance3()*10.0d) & 0x3FF)); 
		   return new short[]{val1, val2, val3, val4, val5};
	}
	private static long[] genChromoSpatialKeyRange(long key) {
		long degThetaL = (key & 0xFFFF000000000000L) >> 48;
		long degPhiL =   (key & 0x0000FFFF00000000L) >> 32;
		long var1L =     (key & 0x00000000FFC00000L) >> 20;
		long var2L =     (key & 0x0000000000FFC00L) >> 10;
		long var3L =     (key & 0x000000000000FFCL);
		//System.out.println("key="+key+" theta= "+degThetaL+" phi= "+degPhiL+" var1= "+var1L+" var2= "+var2L+" var3= "+var3L);
		// deg*100 = 0-36099 = 1000110100000011b
		/*
		long degThetaH = degThetaL + 1;
		degThetaL -= 1;
		if(degThetaL < 0)
			degThetaL = 0;
		if( degThetaH > 36000)
			degThetaH = 36000;
		// TODO:if range > 360 or < 0 handle better
		long degPhiH = degPhiL + 1;
		degPhiL -= 1;
		if(degPhiL < 0)
			degPhiL = 0;
		if( degPhiH > 36000)
			degPhiH = 36000;
		// axis of middle variance, perp to normal
		long var1H = var1L + 1;
		var1L -= 1;
		if( var1L < 0)
			var1L = 0;
		if( var1H > 1023)
			var1H = 1023;
		long var2H = var2L + 1;
		var2L -= 1;
		if( var2L < 0)
			var2L = 0;
		if( var2H > 1023)
			var2H = 1023;
		long var3H = var3L + 1;
		var3L -= 1;
		if( var3L < 0)
			var3L = 0;
		if( var3H > 1023)
			var3H = 1023;
		*/
		long degThetaH = degThetaL + 1;
		degThetaL -= 1;
		if(degThetaL < 0)
			degThetaL = 0;
		if( degThetaH > 36099)
			degThetaH = 36099;
		degPhiL = 0;
		long degPhiH = 36000;
		var1L = 0;
		long var1H = 0x33F;
		var2L = 0;
		long var2H = 0x33F;
		var3L = 0;
		long var3H = 0x33F;
		// 0x3FF 10 bits 1023 or 102.3 max variance
		long val1 = ((degThetaL & 0xFFFF) << 48) | ((degPhiL & 0xFFFF) << 32) | 
				   ((var1L & 0x3FF) << 20) | ((var2L & 0x3FF) << 10) | ((var3L & 0x3FF)); 
		long val2 = ((degThetaH & 0xFFFF) << 48) | ((degPhiH & 0xFFFF) << 32) | 
				   ((var1H & 0x3FF) << 20) | ((var2H & 0x3FF) << 10) | ((var3H & 0x3FF)); 
		//System.out.println("keylow="+val1+" keyhigh="+val2);
		return new long[]{val1, val2};
	}
	/**
	 * Gen the RMS error for the 2 DCT arrays of left and right pixel block colors sans luminance
	 * @param inode
	 * @param jnode
	 * @param nsize
	 * @return
	 */
	private static double genDCT2(octree_t inode, octree_t jnode, int nsize) {
		   float[] coeffsi = new float[nsize];
		   float[] coeffsj = new float[nsize];
		   int i = 0;
		   for(int c = 0; c < nsize; c++) {
				Vector4d lcolor = inode.getRoot().m_colors.get(inode.getIndexes().get(c));
				Vector4d rcolor = jnode.getRoot().m_colors.get(jnode.getIndexes().get(c));				
				coeffsi[c] = (int)lcolor.x<<16 | (int)lcolor.y<<8 | (int)lcolor.z;
				coeffsj[c] = (int)rcolor.x<<16 | (int)rcolor.y<<8 | (int)rcolor.z;
		   }
		   FloatDCT_1D fdct1d = new FloatDCT_1D(nsize);
		   fdct1d.forward(coeffsi, false);
		   fdct1d.forward(coeffsj, false);
		   return IOUtils.computeRMSE(coeffsi, coeffsj);
	}
	
	private static double genSAD(octree_t inode, octree_t jnode, int nsize) {
		double osum = 0;
		for(int c = 0; c < nsize; c++) {
			Vector4d lcolor = inode.getRoot().m_colors.get(inode.getIndexes().get(c));
			Vector4d rcolor = jnode.getRoot().m_colors.get(jnode.getIndexes().get(c));				
			osum += (Math.abs((int)lcolor.x-(int)rcolor.x)<<16 |
					Math.abs((int)lcolor.y-(int)rcolor.y)<<8 |
					Math.abs((int)lcolor.z+(int)rcolor.z));
		}
		return osum;
	}
	/**
	 * Generate the occupancy grid with minimum depth indicators into file seq
	 * from list of IndexMax elements
	 * @param nodes
	 */
	private static void genNav2(List<envInterface> nodes) {
		// descending sort mirrors forward direction of robot movement
		   //final Comparator<envInterface> yComp = new Comparator<envInterface>() {         
			//	  @Override         
			//	  public int compare(envInterface jc1, envInterface jc2) {             
			//		      return ((int)((IndexMax)jc2).node.getMiddle().y < (int)((IndexMax)jc1).node.getMiddle().y ? -1 :                     
			//		              ((int)((IndexMax)jc2).node.getMiddle().y == (int)((IndexMax)jc1).node.getMiddle().y ? 0 : 1));           
			//	  }     
		   //};
		   final Comparator<double[]> yComp = new Comparator<double[]>() {         
				  @Override         
				  public int compare(double[] jc1, double[] jc2) {             
					      return ((int)jc2[3] < (int)jc1[3] ? -1 :                     
					              ((int)jc2[3] == (int)jc1[3] ? 0 : 1));           
				  }     
		   };
		   final Comparator<envInterface> xComp = new Comparator<envInterface>() {         
				  @Override         
				  public int compare(envInterface jc1, envInterface jc2) {             
					      return ((int)((IndexMax)jc2).node.getMiddle().x < (int)((IndexMax)jc1).node.getMiddle().x ? -1 :                     
					              ((int)((IndexMax)jc2).node.getMiddle().x == (int)((IndexMax)jc1).node.getMiddle().x ? 0 : 1));           
				  }     
		   };
		   if(nodes.size() == 0)
			   return;
		   DataOutputStream dos = null;
		   File f = new File(hough_settings.file+"NavLine"+files+hough_settings.extension);
		   try {
			   dos = new DataOutputStream(new FileOutputStream(f));
			   // col and min depth marker
			   ArrayList<double[]> splinePts = new ArrayList<double[]>();
			   Collections.sort(nodes, xComp);
			   int x = (int)((IndexMax)nodes.get(0)).node.getMiddle().x;
			   int iPosStart = 0;
			   int iPosEnd = 0;
			   for(int i = 0 ; i < nodes.size(); i++) {
				   if( x != (int)((IndexMax)nodes.get(i)).node.getMiddle().x) {
					   iPosEnd = i;
					   genColHist2(dos, iPosStart, iPosEnd, nodes, splinePts);
					   iPosStart = i;
					   x = (int)((IndexMax)nodes.get(i)).node.getMiddle().x;
				   }
			   }
			   iPosEnd = nodes.size();
			   genColHist2(dos, iPosStart, iPosEnd, nodes, splinePts);
			   if(splinePts.size() == 0)
				   return;
			   // rows
			   //Collections.sort(nodes, yComp);
			   //int y = (int)((IndexMax)nodes.get(0)).node.getMiddle().y;
			   //iPosStart = 0;
			   //iPosEnd = 0;
			   //for(int i = 0 ; i < nodes.size(); i++) {
				//   if( y != (int)((IndexMax)nodes.get(i)).node.getMiddle().y) {
				//	   iPosEnd = i;
				//	   genRow2(dos, iPosStart, iPosEnd, nodes);
				//	   iPosStart = i;
				//	   y = (int)((IndexMax)nodes.get(i)).node.getMiddle().y;
				//   }
			   //}
			   //iPosEnd = nodes.size();
			   Collections.sort(splinePts, yComp);
			   int y = (int)splinePts.get(0)[3]; //y
			   iPosStart = 0;
			   iPosEnd = 0;
			   for(int i = 0 ; i < splinePts.size(); i++) {
				   if( y != (int)splinePts.get(i)[3]) {
					   iPosEnd = i;
					   genRow2(dos, iPosStart, iPosEnd, splinePts);
					   iPosStart = i;
					   y = (int)splinePts.get(i)[3];
				   }
			   }
			   iPosEnd = splinePts.size();
			   genRow2(dos, iPosStart, iPosEnd, splinePts);
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
				} catch (IOException e) {}		
		  }
		   
	}

	public static void genRow2(DataOutputStream dos, int yStart, int yEnd, List<double[]> splinePts) throws IOException{
		//if(DEBUG)
			//System.out.println("genRow from="+yStart+" to="+yEnd);
			List<double[]> subNodes = splinePts.subList(yStart, yEnd);
			//final Comparator<envInterface> xComp = new Comparator<envInterface>() {         
			//		  @Override         
			//		  public int compare(envInterface jc1, envInterface jc2) {             
			//			      return ((int)((IndexMax)jc2).node.getMiddle().x < (int)((IndexMax)jc1).node.getMiddle().x ? -1 :                     
			//			              ((int)((IndexMax)jc2).node.getMiddle().x == (int)((IndexMax)jc1).node.getMiddle().x ? 0 : 1));           
			//		  }     
			//};
			final Comparator<double[]> xComp = new Comparator<double[]>() {         
					  @Override         
					  public int compare(double[] jc1, double[] jc2) {             
						      return ((int)jc2[0] < (int)jc1[0] ? -1 :                     
						              ((int)jc2[0] == (int)jc1[0] ? 0 : 1));           
					  }     
			};
			Collections.sort(subNodes, xComp);
			// sorted now by x and y
			double zMin = Double.MAX_VALUE;
			double[] cnode = null;
			for(int i = 0; i < subNodes.size(); i++) {
				//System.out.println("Row "+(int)subNodes.get(i)[0]+" "+(int)subNodes.get(i)[1]+" "+(int)subNodes.get(i)[2]);
				if( i < subNodes.size()-1) {
					//writer_file.line3D(dos, (int)((IndexMax)subNodes.get(i)).node.getCentroid().x, (int)((IndexMax)subNodes.get(i)).node.getCentroid().y, (int)((IndexMax)subNodes.get(i)).depth*10,
					//		(int)((IndexMax)subNodes.get(i+1)).node.getCentroid().x, (int)((IndexMax)subNodes.get(i+1)).node.getCentroid().y, (int)((IndexMax)subNodes.get(i+1)).depth*10,
					//		0, 255, 255);
					writer_file.line3D(dos, (int)subNodes.get(i)[0] , (int)subNodes.get(i)[1], (int)subNodes.get(i)[2], 
							(int)subNodes.get(i+1)[0], (int)subNodes.get(i+1)[1], (int)subNodes.get(i+1)[2], 0, 255, 255);
				}
				if((int)subNodes.get(i)[1] >= 0 && (int)subNodes.get(i)[0] >= leftBound && (int)subNodes.get(i)[0] <= rightBound &&
						subNodes.get(i)[2] <= zMin) {
					zMin = subNodes.get(i)[2];
					cnode = subNodes.get(i);
				}
			}
			//System.out.println("-----------");
			//  min depth
			if( cnode != null ) {
			Vector4d cen1 = new Vector4d();
			Vector4d cen2 = new Vector4d();
			cen1.x = cnode[0] - (cnode[4]/2);
			cen1.y = cnode[1] - (cnode[4]/2);
			cen1.z = zMin;
			cen2.x = cnode[0] + (cnode[4]/2);
			cen2.y = cnode[1] + (cnode[4]/2);
			cen2.z = zMin;
			//if( DEBUG )
			//	System.out.println("genColHist env minx,y,z="+cen1+" maxx,y,z="+cen2+" centroid node="+cnode);
			// xmin, ymin, xmax, ymin
			writer_file.line3D(dos, (int)cen1.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen1.y, (int)cen1.z, 255, 127, 0);
			// xmax, ymin, xmax, ymax
			writer_file.line3D(dos, (int)cen2.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen2.y, (int)cen2.z, 255, 127, 0);
			// xmax, ymax, xmin, ymax
			writer_file.line3D(dos, (int)cen2.x, (int)cen2.y, (int)cen2.z, (int)cen1.x, (int)cen2.y, (int)cen2.z, 255, 127, 0);
			// xmin, ymax, xmin, ymin
			writer_file.line3D(dos, (int)cen1.x, (int)cen2.y, (int)cen2.z, (int)cen1.x, (int)cen1.y, (int)cen1.z, 255, 127, 0);
			}
	}
	public static void genColHist2(DataOutputStream dos, int xStart, int xEnd, List<envInterface> nodelA, List<double[]> splinePts) throws IOException{
		//if(DEBUG)
			//System.out.println("genColHist from="+xStart+" to="+xEnd);
			List<envInterface> subNodes = nodelA.subList(xStart, xEnd);
			final Comparator<envInterface> yComp = new Comparator<envInterface>() {         
					  @Override         
					  public int compare(envInterface jc1, envInterface jc2) {             
						      return ((int)((IndexMax)jc2).node.getMiddle().y < (int)((IndexMax)jc1).node.getMiddle().y ? -1 :                     
						              ((int)((IndexMax)jc2).node.getMiddle().y == (int)((IndexMax)jc1).node.getMiddle().y ? 0 : 1));           
					  }     
			}; 
			Collections.sort(subNodes, yComp);
			ArrayList<double[]> dpit = new ArrayList<double[]>();
			if(SMOOTHEGRID) {
				PolyBezierPathUtil pbpu = new  PolyBezierPathUtil();
				ArrayList<PolyBezierPathUtil.EPointF> epfa = new ArrayList<PolyBezierPathUtil.EPointF>();
				for(int i = 0; i < subNodes.size(); i++) {
					PolyBezierPathUtil.EPointF epf = pbpu.new EPointF(((IndexMax)subNodes.get(i)).depth*10.0d, ((IndexMax)subNodes.get(i)).node.getCentroid().y);
					epfa.add(epf);
				}
				if(epfa.size() < 2)
					return;
				Path2D.Double path = pbpu.computePathThroughKnots(epfa);
				PathIterator pit = path.getPathIterator(null);
				while(!pit.isDone()) {
					double[] coords = new double[6];
					pit.currentSegment(coords);
					dpit.add(coords);
					pit.next();
				}
			} else {
				//---if we dont want to smooth
				for(int i = 0; i < subNodes.size(); i++) {
					double[] coords = new double[6];
					coords[0] = ((IndexMax)subNodes.get(i)).depth*10.0d;
					coords[1] = ((IndexMax)subNodes.get(i)).node.getCentroid().y;
					dpit.add(coords);
				}
			}
			double zMin = Double.MAX_VALUE;
			octree_t cnode = null;
			int cpos = -1;
			// sorted now by x and y
			for(int i = 0; i < subNodes.size(); i++) {
				double[] splinep = dpit.get(i);
				splinep[2] = splinep[0]; // rotate X back to Z
				splinep[0] = ((IndexMax)subNodes.get(i)).node.getCentroid().x;// the x, then y is index 1 as originally
				splinep[3] = ((IndexMax)subNodes.get(i)).node.getMiddle().y;// we need to sort the rows on this value
				splinep[4] = ((IndexMax)subNodes.get(i)).node.getSize(); // to generate box
				splinePts.add(splinep); // collect it in our splined array to make rows later
				if( i < subNodes.size()-1) {
					//writer_file.line3D(dos, (int)((IndexMax)subNodes.get(i)).node.getCentroid().x, (int)((IndexMax)subNodes.get(i)).node.getCentroid().y, (int)((IndexMax)subNodes.get(i)).depth*10,
					//		(int)((IndexMax)subNodes.get(i+1)).node.getCentroid().x, (int)((IndexMax)subNodes.get(i+1)).node.getCentroid().y, (int)((IndexMax)subNodes.get(i+1)).depth*10,
					//		0, 255, 255);
					writer_file.line3D(dos, (int)splinep[0] , (int)splinep[1], (int)splinep[2], 
							(int)((IndexMax)subNodes.get(i+1)).node.getCentroid().x, (int)dpit.get(i+1)[1], (int)dpit.get(i+1)[0], 0, 255, 255);
					//if( DEBUG ) 
					//	System.out.println("genColHist line3D x="+(int)((IndexMax)subNodes.get(i)).node.getCentroid().x+" y="+(int)((IndexMax)subNodes.get(i)).node.getCentroid().y+" z="+(int)((IndexMax)subNodes.get(i)).depth*10+
					//			" to x="+(int)((IndexMax)subNodes.get(i+1)).node.getCentroid().x+" y="+(int)((IndexMax)subNodes.get(i+1)).node.getCentroid().y+" z="+(int)((IndexMax)subNodes.get(i+1)).depth*10);
					// y remains the same and we rotate our depth 90 degrees to become the x axis
					// so we get an angular measurement thats perpendicular to frontal x,y plane
					// this corresponds to the theta angle we get from cartestian_to_spherical, for some reason in this
					// orientation, vs z axis vertical, the theta value is aligned along our depth, or z axis horizontal. 
					// So if we use spherical the phi and theta are swapped in this orientation.
					//double thet1 = Math.atan2( (((IndexMax)subNodes.get(i)).node.getCentroid().y-((IndexMax)subNodes.get(i+1)).node.getCentroid().y),
					//			((((IndexMax)subNodes.get(i)).depth-((IndexMax)subNodes.get(i+1)).depth)*10) );
					//double thet1 = Math.atan2( splinep[1]-dpit.get(i+1)[1], ((splinep[2]-dpit.get(i+1)[0])*10) );
					//double degThet = Math.toDegrees(thet1);
					//if( degThet < 0.0) {
					//	    degThet += 360.0;
					//}
					//Vector4d point = new Vector4d( (((IndexMax)subNodes.get(i)).node.getCentroid().x-((IndexMax)subNodes.get(i+1)).node.getCentroid().x),
					//		(((IndexMax)subNodes.get(i)).node.getCentroid().y-((IndexMax)subNodes.get(i+1)).node.getCentroid().y),
					//			((((IndexMax)subNodes.get(i)).depth-((IndexMax)subNodes.get(i+1)).depth)*10), 1);
					//double[] deg1 = octree_t.cartesian_to_spherical(point);
					//
					//System.out.println("Column "+(int)((IndexMax)subNodes.get(i)).node.getMiddle().x+" depth diff "+i+" degrees ="+degThet+" node="+((IndexMax)subNodes.get(i)).node);
				}
				// since we sort descending to compute in the direction of robot motion, we want the highest numbered
				// row as minimum so we know the top of an obstacle right in front of us. To this end we use <= zMin
				// so identical depths will percolate to the top
				if((int)((IndexMax)subNodes.get(i)).node.getCentroid().y >= 0 && (int)splinep[0] >= leftBound && (int)splinep[0] <= rightBound &&
						((IndexMax)subNodes.get(i)).depth <= zMin) {
					zMin = ((IndexMax)subNodes.get(i)).depth;
					//cnode = ((IndexMax)subNodes.get(i)).node;
					cpos = i;
				}
			}
			//System.out.println("----------"); //delinate column depth display
			//  min depth
			//if( cnode != null ) {
			if( cpos != -1 ) { // there may have been 0 elements
				for(int i = cpos; i > 0; i--) {
					cnode = ((IndexMax)subNodes.get(i)).node;	
					Vector4d cen1 = new Vector4d();
					Vector4d cen2 = new Vector4d();
					cen1.x = cnode.getCentroid().x - (cnode.getSize()/2);
					cen1.y = cnode.getCentroid().y - (cnode.getSize()/2);
					cen1.z = zMin*10.0d;//cnode.getCentroid().z - (cnode.getSize()/2);
					cen2.x = cnode.getCentroid().x + (cnode.getSize()/2);
					cen2.y = cnode.getCentroid().y + (cnode.getSize()/2);
					cen2.z = zMin*10.0d;//cnode.getCentroid().z + (cnode.getSize()/2);
					//if( DEBUG )
					//	System.out.println("genColHist env minx,y,z="+cen1+" maxx,y,z="+cen2+" centroid node="+cnode);
					// xmin, ymin, xmax, ymin
					writer_file.line3D(dos, (int)cen1.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen1.y, (int)cen1.z, 0, 127, 255);
					// xmax, ymin, xmax, ymax
					writer_file.line3D(dos, (int)cen2.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen2.y, (int)cen2.z, 0, 127, 255);
					// xmax, ymax, xmin, ymax
					writer_file.line3D(dos, (int)cen2.x, (int)cen2.y, (int)cen2.z, (int)cen1.x, (int)cen2.y, (int)cen2.z, 0, 127, 255);
					// xmin, ymax, xmin, ymin
					writer_file.line3D(dos, (int)cen1.x, (int)cen2.y, (int)cen2.z, (int)cen1.x, (int)cen1.y, (int)cen1.z, 0, 127, 255);
				}
			}
	}
	/**
	 * Permutation that does not rely on octree node
	 * @param nodes
	 */
	public static void genNav(List<envInterface> nodes) {
		   final Comparator<envInterface> yComp = new Comparator<envInterface>() {         
				  @Override         
				  public int compare(envInterface jc1, envInterface jc2) {
					  int y1 = (int)(jc1.getEnv()[1]+((jc1.getEnv()[3]-jc1.getEnv()[1])/2));
					  int y2 = (int)(jc2.getEnv()[1]+((jc2.getEnv()[3]-jc2.getEnv()[1])/2));
					      return (y2 < y1 ? -1 : (y2 == y1 ? 0 : 1));           
				  }     
		   };
		   final Comparator<envInterface> xComp = new Comparator<envInterface>() {         
				  @Override         
				  public int compare(envInterface jc1, envInterface jc2) {
					  int x1 = (int)(jc1.getEnv()[0]+((jc1.getEnv()[2]-jc1.getEnv()[0])/2));
					  int x2 = (int)(jc2.getEnv()[0]+((jc2.getEnv()[2]-jc2.getEnv()[0])/2));
					      return (x2 < x1 ? -1 : (x2 == x1 ? 0 : 1));           
				  }     
		   };
		   Collections.sort(nodes, yComp);
		   DataOutputStream dos = null;
		   File f = new File(hough_settings.file+"NavLine"+files+hough_settings.extension);
		   try {
			   dos = new DataOutputStream(new FileOutputStream(f));
			   int y = (int)(nodes.get(0).getEnv()[1]+((nodes.get(0).getEnv()[3]-nodes.get(0).getEnv()[1])/2));
			   int iPosStart = 0;
			   int iPosEnd = 0;
			   for(int i = 0 ; i < nodes.size(); i++) {
				   if( y != (int)(nodes.get(i).getEnv()[1]+((nodes.get(i).getEnv()[3]-nodes.get(i).getEnv()[1])/2))) {
					   iPosEnd = i;
					   genRow(dos, iPosStart, iPosEnd, nodes);
					   iPosStart = i;
					   y = (int)(nodes.get(i).getEnv()[1]+((nodes.get(i).getEnv()[3]-nodes.get(i).getEnv()[1])/2));
				   }
			   }
			   iPosEnd = nodes.size();
			   genRow(dos, iPosStart, iPosEnd, nodes);
			   // col and min depth marker
			   Collections.sort(nodes, xComp);
			   int x = (int)(nodes.get(0).getEnv()[0]+((nodes.get(0).getEnv()[2]-nodes.get(0).getEnv()[0])/2));
			   iPosStart = 0;
			   iPosEnd = 0;
			   for(int i = 0 ; i < nodes.size(); i++) {
				   if( x != nodes.get(i).getEnv()[0]+((nodes.get(i).getEnv()[2]-nodes.get(i).getEnv()[0])/2)) {
					   iPosEnd = i;
					   genColHist(dos, iPosStart, iPosEnd, nodes);
					   iPosStart = i;
					   x =(int)(nodes.get(i).getEnv()[0]+((nodes.get(i).getEnv()[2]-nodes.get(i).getEnv()[0])/2));
				   }
			   }
			   iPosEnd = nodes.size();
			   genColHist(dos, iPosStart, iPosEnd, nodes);
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
				} catch (IOException e) {}		
		  }   
	}

	public static void genRow(DataOutputStream dos, int yStart, int yEnd, List<envInterface> nodelA) throws IOException{
		//if(DEBUG)
		//	System.out.println("genRow from="+yStart+" to="+yEnd);
			List<envInterface> subNodes = nodelA.subList(yStart, yEnd);
			final Comparator<envInterface> xComp = new Comparator<envInterface>() {         
					  @Override         
					  public int compare(envInterface jc1, envInterface jc2) {
						  int x1 = (int)(jc1.getEnv()[0]+((jc1.getEnv()[2]-jc1.getEnv()[0])/2));
						  int x2 = (int)(jc2.getEnv()[0]+((jc2.getEnv()[2]-jc2.getEnv()[0])/2));
						      return (x2 < x1 ? -1 : (x2 == x1 ? 0 : 1));           
					  }     
			};
			Collections.sort(subNodes, xComp);
			// sorted now by x and y
			for(int i = 0; i < subNodes.size(); i++) {
				if( i < subNodes.size()-1) {
					int ix0 = (int)subNodes.get(i).getEnv()[0]+(((int)subNodes.get(i).getEnv()[2]-(int)subNodes.get(i).getEnv()[0])/2);
					int iy0 = (int)subNodes.get(i).getEnv()[1]+(((int)subNodes.get(i).getEnv()[3]-(int)subNodes.get(i).getEnv()[1])/2);
					int iz0 = (int)(subNodes.get(i).getDepth()*10.0d);
					int ix1 = (int)subNodes.get(i+1).getEnv()[0]+(((int)subNodes.get(i+1).getEnv()[2]-(int)subNodes.get(i+1).getEnv()[0])/2);
					int iy1 = (int)subNodes.get(i+1).getEnv()[1]+(((int)subNodes.get(i+1).getEnv()[3]-(int)subNodes.get(i+1).getEnv()[1])/2);
					int iz1 = (int)(subNodes.get(i+1).getDepth()*10.0d);
					writer_file.line3D(dos, ix0, iy0, iz0, ix1, iy1, iz1, 0, 255, 255);
				}
			}
	}
	
	public static void genColHist(DataOutputStream dos, int xStart, int xEnd, List<envInterface> nodelA) throws IOException{
		//if(DEBUG)
			//System.out.println("genColHist from="+xStart+" to="+xEnd);
			List<envInterface> subNodes = nodelA.subList(xStart, xEnd);
			final Comparator<envInterface> yComp = new Comparator<envInterface>() {         
					  @Override         
					  public int compare(envInterface jc1, envInterface jc2) {
						  int y1 = (int)(jc1.getEnv()[1]+((jc1.getEnv()[3]-jc1.getEnv()[1])/2));
						  int y2 = (int)(jc2.getEnv()[1]+((jc2.getEnv()[3]-jc2.getEnv()[1])/2));
						      return (y2 < y1 ? -1 : (y2 == y1 ? 0 : 1));           
					  }     
			};
			Collections.sort(subNodes, yComp);
			double zMin = Double.MAX_VALUE;
			int ix0 = 0;
			int iy0 = 0;
			int iz0 = 0;
			int ix1 = 0;
			int iy1 = 0;
			int iz1 = 0;
			envInterface cnode = null;
			// sorted now by x and y
			for(int i = 0; i < subNodes.size(); i++) {
				if( i < subNodes.size()-1) {
					ix0 = (int)subNodes.get(i).getEnv()[0]+(((int)subNodes.get(i).getEnv()[2]-(int)subNodes.get(i).getEnv()[0])/2);
					iy0 = (int)subNodes.get(i).getEnv()[1]+(((int)subNodes.get(i).getEnv()[3]-(int)subNodes.get(i).getEnv()[1])/2);
					iz0 = (int)(subNodes.get(i).getDepth()*10.0d);
					ix1 = (int)subNodes.get(i+1).getEnv()[0]+(((int)subNodes.get(i+1).getEnv()[2]-(int)subNodes.get(i+1).getEnv()[0])/2);
					iy1 = (int)subNodes.get(i+1).getEnv()[1]+(((int)subNodes.get(i+1).getEnv()[3]-(int)subNodes.get(i+1).getEnv()[1])/2);
					iz1 = (int)(subNodes.get(i+1).getDepth()*10.0d);
					writer_file.line3D(dos, ix0, iy0, iz0, ix1, iy1, iz1, 0, 255, 255);
					//if( DEBUG ) 
					//	System.out.println("genColHist line3D x="+ix0+" y="+iy0+" z="+iz0+" to x="+iz1+" y="+iy1+" z="+iz1);
				}
				if(iy0 >= 0 && subNodes.get(i).getDepth() < zMin) {
					zMin = subNodes.get(i).getDepth();
					cnode = subNodes.get(i);
				}
			}
			//  min depth
			if( cnode != null ) {
			Vector4d cen1 = new Vector4d();
			Vector4d cen2 = new Vector4d();
			cen1.x = cnode.getEnv()[0];
			cen1.y = cnode.getEnv()[1];
			cen1.z = cnode.getDepth()*10.0d;//cnode.getCentroid().z - (cnode.getSize()/2);
			cen2.x = cnode.getEnv()[2];
			cen2.y = cnode.getEnv()[3];
			cen2.z = cnode.getDepth()*10.0d;//cnode.getCentroid().z + (cnode.getSize()/2);
			//if( DEBUG )
			//	System.out.println("genColHist env minx,y,z="+cen1+" maxx,y,z="+cen2+" centroid node="+cnode);
			// xmin, ymin, xmax, ymin
			writer_file.line3D(dos, (int)cen1.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen1.y, (int)cen1.z, 0, 127, 255);
			// xmax, ymin, xmax, ymax
			writer_file.line3D(dos, (int)cen2.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen2.y, (int)cen2.z, 0, 127, 255);
			// xmax, ymax, xmin, ymax
			writer_file.line3D(dos, (int)cen2.x, (int)cen2.y, (int)cen2.z, (int)cen1.x, (int)cen2.y, (int)cen2.z, 0, 127, 255);
			// xmin, ymax, xmin, ymin
			writer_file.line3D(dos, (int)cen1.x, (int)cen2.y, (int)cen2.z, (int)cen1.x, (int)cen1.y, (int)cen1.z, 0, 127, 255);
			}
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
					writer_file.line3D(dos, (int)id.getEnv()[0]/*xmin*/, (int)id.getEnv()[1]/*ymin*/, (int)(id.getDepth()*10.0d), 
							(int)id.getEnv()[2]/*xmax*/, (int)id.getEnv()[1]/*ymin*/, (int)(id.getDepth()*10.0d), 0, 255, 255);
					writer_file.line3D(dos, (int)id.getEnv()[2]/*xmax*/, (int)id.getEnv()[1]/*ymin*/, (int)(id.getDepth()*10.0d), 
							(int)id.getEnv()[2]/*xmax*/, (int)id.getEnv()[3]/*ymax*/, (int)(id.getDepth()*10.0d), 0, 255, 255);
					writer_file.line3D(dos, (int)id.getEnv()[2]/*xmax*/, (int)id.getEnv()[3]/*ymax*/, (int)(id.getDepth()*10.0d), 
							(int)id.getEnv()[0]/*xmin*/, (int)id.getEnv()[3]/*ymax*/, (int)(id.getDepth()*10.0d), 0, 255, 255);
					writer_file.line3D(dos, (int)id.getEnv()[0]/*xmin*/, (int)id.getEnv()[3]/*ymax*/, (int)(id.getDepth()*10.0d), 
							(int)id.getEnv()[0]/*xmin*/, (int)id.getEnv()[1]/*ymin*/, (int)(id.getDepth()*10.0d), 0, 255, 255);
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
		
		protected void writeTestPair(envInterface d, envInterface dd, String filename) {
			DataOutputStream dos = null;
			File f = new File(outDir+filename+".asc");
			try {
				dos = new DataOutputStream(new FileOutputStream(f));
					writer_file.line3D(dos, (int)d.getEnv()[0]/*xmin*/, (int)d.getEnv()[1]/*ymin*/, (int)(d.getDepth()*10.0d), 
							(int)d.getEnv()[2]/*xmax*/, (int)d.getEnv()[1]/*ymin*/,  (int)(d.getDepth()*10.0d), 0, 255, 255);
					writer_file.line3D(dos, (int)d.getEnv()[2]/*xmax*/, (int)d.getEnv()[1]/*ymin*/,  (int)(d.getDepth()*10.0d), 
							(int)d.getEnv()[2]/*xmax*/, (int)d.getEnv()[3]/*ymax*/,  (int)(d.getDepth()*10.0d), 0, 255, 255);
					writer_file.line3D(dos, (int)d.getEnv()[2]/*xmax*/, (int)d.getEnv()[3]/*ymax*/,  (int)(d.getDepth()*10.0d), 
							(int)d.getEnv()[0]/*xmin*/, (int)d.getEnv()[3]/*ymax*/, (int)(d.getDepth()*10.0d), 0, 255, 255);
					writer_file.line3D(dos, (int)d.getEnv()[0]/*xmin*/, (int)d.getEnv()[3]/*ymax*/,  (int)(d.getDepth()*10.0d), 
							(int)d.getEnv()[0]/*xmin*/, (int)d.getEnv()[1]/*ymin*/,  (int)(d.getDepth()*10.0d), 0, 255, 255);
					//
					writer_file.line3D(dos, (int)dd.getEnv()[0]/*xmin*/, (int)dd.getEnv()[1]/*ymin*/,  (int)(d.getDepth()*10.0d), 
							(int)dd.getEnv()[2]/*xmax*/, (int)dd.getEnv()[1]/*ymin*/,  (int)(d.getDepth()*10.0d), 255, 255, 0);
					writer_file.line3D(dos, (int)dd.getEnv()[2]/*xmax*/, (int)dd.getEnv()[1]/*ymin*/,  (int)(d.getDepth()*10.0d), 
							(int)dd.getEnv()[2]/*xmax*/, (int)dd.getEnv()[3]/*ymax*/,  (int)(d.getDepth()*10.0d), 255, 255, 0);
					writer_file.line3D(dos, (int)dd.getEnv()[2]/*xmax*/, (int)dd.getEnv()[3]/*ymax*/,  (int)(d.getDepth()*10.0d), 
							(int)dd.getEnv()[0]/*xmin*/, (int)dd.getEnv()[3]/*ymax*/, (int)(d.getDepth()*10.0d), 255, 255, 0);
					writer_file.line3D(dos, (int)dd.getEnv()[0]/*xmin*/, (int)dd.getEnv()[3]/*ymax*/,  (int)(d.getDepth()*10.0d), 
							(int)dd.getEnv()[0]/*xmin*/, (int)dd.getEnv()[1]/*ymin*/,  (int)(d.getDepth()*10.0d), 255, 255, 0);
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
		
		public void writeTestPerp(IndexDepth d1, IndexDepth d2, String filename) {
			DataOutputStream dos = null;
			File f = new File(outDir+filename+".asc");
			try {
				dos = new DataOutputStream(new FileOutputStream(f));
				Vector4d cen1 = new Vector4d();
				Vector4d cen2 = new Vector4d();
				Vector4d cen3 = new Vector4d();
				Vector4d cen4 = new Vector4d();
				writer_file.genPerp(d1.node, cen1, cen2, cen3, cen4);
				writer_file.line3D(dos, (int)d1.node.getCentroid().x, (int)d1.node.getCentroid().y, (int)d1.node.getCentroid().z, (int)cen1.x, (int)cen1.y, (int)cen1.z, 255, 255, 255);
				// axis of middle variance, perp to normal
				writer_file.line3D(dos, (int)cen1.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen2.y, (int)cen2.z, 255, 0, 0);
				// long axis
				// generate point at end of long segment
				writer_file.line3D(dos, (int)cen3.x, (int)cen3.y, (int)cen3.z, (int)cen4.x, (int)cen4.y, (int)cen4.z, 0, 0, 255);
				// outside
				writer_file.line3D(dos, (int)cen1.x, (int)cen1.y, (int)cen1.z, (int)cen4.x, (int)cen4.y, (int)cen4.z, 0, 255, 0);
				writer_file.line3D(dos, (int)cen4.x, (int)cen4.y, (int)cen4.z, (int)cen2.x, (int)cen2.y, (int)cen2.z, 0, 255, 0);
				writer_file.line3D(dos, (int)cen2.x, (int)cen2.y, (int)cen2.z, (int)cen3.x, (int)cen3.y, (int)cen3.z, 0, 255, 0);
				writer_file.line3D(dos, (int)cen3.x, (int)cen3.y, (int)cen3.z, (int)cen1.x, (int)cen1.y, (int)cen1.z, 0, 255, 0);
				writer_file.genPerp(d2.node, cen1, cen2, cen3, cen4);
				writer_file.line3D(dos, (int)d2.node.getCentroid().x, (int)d2.node.getCentroid().y, (int)d2.node.getCentroid().z, (int)cen1.x, (int)cen1.y, (int)cen1.z, 255, 255, 255);
				// axis of middle variance, perp to normal
				writer_file.line3D(dos, (int)cen1.x, (int)cen1.y, (int)cen1.z, (int)cen2.x, (int)cen2.y, (int)cen2.z, 255, 0, 0);
				// long axis
				// generate point at end of long segment
				writer_file.line3D(dos, (int)cen3.x, (int)cen3.y, (int)cen3.z, (int)cen4.x, (int)cen4.y, (int)cen4.z, 0, 0, 255);
				// outside
				writer_file.line3D(dos, (int)cen1.x, (int)cen1.y, (int)cen1.z, (int)cen4.x, (int)cen4.y, (int)cen4.z, 255, 255, 0);
				writer_file.line3D(dos, (int)cen4.x, (int)cen4.y, (int)cen4.z, (int)cen2.x, (int)cen2.y, (int)cen2.z, 255, 255, 0);
				writer_file.line3D(dos, (int)cen2.x, (int)cen2.y, (int)cen2.z, (int)cen3.x, (int)cen3.y, (int)cen3.z, 255, 255, 0);
				writer_file.line3D(dos, (int)cen3.x, (int)cen3.y, (int)cen3.z, (int)cen1.x, (int)cen1.y, (int)cen1.z, 255, 255, 0);
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
	
	   //---------------------------------
	   // supplementary classes
	   //---------------------------------
	   
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
			//if(DEBUG)
			//	System.out.println("getEnvelope txmin="+txmin+" tymin="+tymin+" txmax="+txmax+" tymax="+tymax);
			return new double[]{txmin, tymin, txmax, tymax};
		}
		/**
		 * Abstract away operations on an envelope that includes a depth. A plane if you will.
		 * A 2D plane set in 3D.
		 *
		 */
		static public interface envInterface {
			public double[] getEnv();
			public double getDepth();
			public boolean encloses(Vector4d tpoint);
			public boolean encloses(envInterface e);
			public boolean enclosedBy(envInterface e);
			public void setDepth(double depth);
		}
		
		static abstract class AbstractDepth implements envInterface {
			public octree_t node;
			public double xmin, ymin, xmax, ymax;
			public double depth;
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
			 * does the passed square lie within this one, even partially?
			 * is xmin of this between xmin and xmax of passed and ymin of
			 * this between ymin and ymax of passed OR
			 * xmax of this between xmin and xmax of passed and ymax of this
			 * between ymin and ymax of passed
			 * @param env
			 * @return true if passed envelope lies within this one
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
			public abstract boolean enclosedBy(envInterface e);
			@Override
			public boolean equals(Object tind) {
				return ( xmin == ((envInterface)tind).getEnv()[0] && xmax == ((envInterface)tind).getEnv()[2] &&
						 ymin == ((envInterface)tind).getEnv()[1] && ymax == ((envInterface)tind).getEnv()[3]);
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
		/**
		 * Abstraction of processed envelopes with the addition of depth. Middle typically refers to centroid
		 * of point cloud in octree node. size is typically derived from octree node edge length, but can
		 * also include the length of the vector of centroid with magnitude eigenvalue in direction eigenvector, since
		 * eigenvectors are represented by normal vectors, we scale them to centroid with angle eigenvecotr magnitude eigenvalue plus
		 * and eigenvalue minus.
		 * @author jg
		 *
		 */
		final class IndexDepth extends AbstractDepth {
			public Vector4d middle;
			public IndexDepth(octree_t node, double depth) {
				this.node = node;
				this.middle = node.getMiddle();
				xmin = middle.x-(node.getSize()/2);
				xmax = middle.x+(node.getSize()/2);
				ymin = middle.y-(node.getSize()/2);
				ymax = middle.y+(node.getSize()/2);
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
			public String toString() {
				return "IndexDepth xmin="+xmin+" ymin="+ymin+" xmax="+xmax+" ymax="+ymax+" depth="+depth+" middle="+middle;
			}
		}
		/**
		 * This class has a similar structure, but quite different behavior , than the IndexDepth class.
		 * It represents the maximal regions along with the octree node at their core
		 * @author jg
		 *
		 */
		final class IndexMax extends AbstractDepth {
			public IndexMax(octree_t node, double[] env, double depth) {
				this.node = node;
				xmin = env[0];
				xmax = env[2];
				ymin = env[1];
				ymax = env[3];
				this.depth = depth;
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
			public String toString() {
				return "IndexMax xmin="+xmin+" ymin="+ymin+" xmax="+xmax+" ymax="+ymax+" depth="+depth+" node="+node;
			}
			
		}
		/**
		 * Parallel left/right image pixels to processing model generator class:<br/>
		 * To initialize:<br/>
		 *  PixelsToModel modelGen = new PixelsToModel();<br/>
		 *  modelGen.spinGen();<br/>
		 * Then for each iteration of new image set:<br/>
		 * 	modelGen.imageL1 = imageL1;<br/>
		 *	modelGen.imageR1 = imageR1;<br/>
		 *  modelGen.latchOutL.reset(); // reset to synch at barrier<br/>
		 *  modelGen.latchOutR.reset();<br/>
		 *  modelGen.latchL.reset(); // after all resets we wait at these barriers at top of thread processing<br/>
		 *  modelGen.latchR.reset();<br/>
		 *  modelGen.latchL.await(); // these trip the barriers to start the threads<br/>
		 *  modelGen.latchR.await();<br/>
		 *  modelGen.latchOutL.await(); // once these 2 barriers are tripped we are done<br/>
		 *  modelGen.latchOutR.await();<br/>
		 * @author jg
		 *
		 */
		final class PixelsToModel {
			private static final boolean DEBUG = true;
			private int[] dataLx; // left array return from model generation (i.e. canny with magnitudes or mean color generator with means)
			private int[] dataRx; // right model array with magnitudes
			BufferedImage imageL1 = null;
			BufferedImage imageR1 = null;
			CyclicBarrier latchL = new CyclicBarrier(2);
			CyclicBarrier latchOutL = new CyclicBarrier(2);
			CyclicBarrier latchR = new CyclicBarrier(2);
			CyclicBarrier latchOutR = new CyclicBarrier(2);
			/**
			 * Activates two threads, one for left, one for right image from stereo camera setup
			 * and enter loop to run genModel method using barrier synchronization to control execution
			 */
			public void spinGen() {
				if(DEBUG)
					System.out.println("Spinning Left Pixels to Model Generator");
				ThreadPoolManager.getInstance().spin(new Runnable() {
					@Override
					public void run() {			
						while(true) {
							try {
								latchL.await();
								dataLx = genModel(imageL1, dataLp, "MODELGENL");
								if( VideoProcessor.imageIntegrate > 0 ) {
									if( dataLp == null ) {
										dataLp = dataLx;
									}
								} else {
									dataLp = dataLx;
								}
								latchOutL.await();
							} catch (InterruptedException| BrokenBarrierException e) {}
						}
					}
				});
				if(DEBUG)
					System.out.println("Spinning Right Pixels to Model Generator");
				ThreadPoolManager.getInstance().spin(new Runnable() {
					@Override
					public void run() {	
						while(true) {
							try {
								latchR.await();
								dataRx = genModel(imageR1, dataRp, "MODELGENR");
								if( VideoProcessor.imageIntegrate > 0 ) {
									if( dataRp == null ) {
										dataRp = dataRx;
									}
								} else {
									dataRp = dataRx;
								}
								latchOutR.await();
							} catch (InterruptedException | BrokenBarrierException e) {}
						}
					}
				});
			}
			/**
			 * Always return the processed image, but if datap is not null and imageIntegrate has
			 * a value greater than 0 xor it with that as well, for as many iterations as the value of imageIntegrate.
			 * From the caller, do the same check and if datap was null, assign it the returned array.
			 * We are trying to 'build up' the image progressively if doing so will add more fidelity
			 * as in edge generation, but if it wont, as in mean color model the VideoProcessor.imageIntegrate
			 * value will be zero vs a limit number of images we use.
			 * @param img The BufferedImage of our source pixels
			 * @param datap The resulting model generated from image
			 * @param threadGroupName The group name since we are using multiple instances of model generator
			 * @return
			 */
			private int[] genModel(BufferedImage img, int[] datap, String threadGroupName) {
		   	    //ParallelCannyEdgeDetector ced = new ParallelCannyEdgeDetector(threadGroupName);
			    //ced.setLowThreshold(.1f);
			    //ced.setHighThreshold(.2f);
			    //ced.setGaussianKernelRadius(2);
			    //ced.setGaussianKernelWidth(16);
				//---SobelEdgeDetector ced = new SobelEdgeDetector();
			    //ced.setSourceImage(img);
			    //int[] datax = ced.semiProcess();
				MeanColorGenerator mcg = new MeanColorGenerator(img, camWidth, camHeight);
				int[] datax = mcg.mean();
				if( VideoProcessor.imageIntegrate > 0 ) {
					if(datap != null) {
						for(int i = 0; i < datap.length; i++) {
			    			 datap[i] = datap[i] | datax[i];
						}
					}
				}
				// get image mean		
			    return datax;
			}
		}
		
		public static void main(String[] args) throws Exception {
			VideoProcessor vp = new VideoProcessor();
			vp.spinUp();
			BufferedImage imageL1 = ImageIO.read(new File(vp.outDir+"/"+args[0]));
			BufferedImage imageR1 = ImageIO.read(new File(vp.outDir+"/"+args[1]));
			/*
    	    CannyEdgeDetector ced = new CannyEdgeDetector();
    	    ced.setLowThreshold(0.1f);//0.5f
    	    ced.setHighThreshold(0.5f);//1f
    	    ced.setSourceImage(imageL1);
    	    int[] dataLx = ced.semiProcess();
    	    //
    	    ced = new CannyEdgeDetector();
    	    ced.setLowThreshold(0.1f);
    	    ced.setHighThreshold(0.5f);
    	    ced.setSourceImage(imageR1);
    	    int[] dataRx = ced.semiProcess();
    	    */
			MeanColorGenerator mcg = new MeanColorGenerator(imageL1, camWidth, camHeight);
			int[] dataLx = mcg.mean();
			mcg = new MeanColorGenerator(imageR1, camWidth, camHeight);
			int[] dataRx = mcg.mean();
			//
			vp.queueI.addLast(new Object[]{imageL1, imageR1, dataLx, dataRx});			
		}

}


