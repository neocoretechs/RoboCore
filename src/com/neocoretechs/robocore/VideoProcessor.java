package com.neocoretechs.robocore;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.Frame;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.text.DecimalFormat;
import java.util.Map;
import java.util.Vector;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.BrokenBarrierException;
import java.util.concurrent.CyclicBarrier;
import java.util.concurrent.atomic.AtomicInteger;

import javax.imageio.ImageIO;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.SwingConstants;
import javax.swing.SwingUtilities;
import javax.swing.WindowConstants;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import org.ros.concurrent.CircularBlockingDeque;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;
import org.jtransforms.dct.FloatDCT_2D;
import org.jtransforms.utils.IOUtils;

import com.neocoretechs.machinevision.CannyEdgeDetector;
import com.neocoretechs.machinevision.HoughElem;
import com.neocoretechs.machinevision.HoughLine;
import com.neocoretechs.machinevision.HoughLine3;
import com.neocoretechs.machinevision.HoughTransform;
import com.neocoretechs.machinevision.HoughTransform3;
import com.neocoretechs.machinevision.ImgProcessor;


/**
 * Create a disparity map for the left and right images taken from stereo cameras published to bus.
 * The methodology is a variation of standard stereo block matching wherein we create a correspondence window
 * of corrWinSize. 
 * We perform a forward discrete cosine transform on the 2 windows to push the major features. 
 * We then use standard SAD (sum of absolute differences) on the DCT arrays to discover
 * a correlation to which the disparity calculation of (focal length * baseline)/ (XRight - XLeft) is applied.
 * We assign the depth values to a corrWinSize Y band 1 x pixel wide. A 1 pixel wide corrWinLength in Y band is the minimum
 * we can assign comparing matrixes of rightmost increasing scans. We refer to the left image pixel moving in X as 'subject' and
 * right image scan in X as 'target'.
 * We break the image into height/corrWinSize bands and assign each band to a processing thread. 
 * We have an option to write a file that can be read by CloudCompare for testing.
 *  __mode:= option on commandline determines how we process, such as 'edge','display', 'display3D',etc.
 * @author jg (C) NeoCoreTechs 2018,2019
 *
 */
public class VideoProcessor extends AbstractNodeMain 
{
	private static boolean DEBUG = false;
	private static final boolean SAMPLERATE = true; // display pubs per second

    private BufferedImage imageL = null;
    private BufferedImage imageR = null;
    private BufferedImage imageLx = null;
    private BufferedImage imageRx = null;
    private BufferedImage imageT = null;
    private PlayerFrame displayPanel;
    private boolean viewChanged = false;
    
    // slider to control horizontal rotation
    final JSlider headingSlider = new JSlider(-180, 180, 0);
    // slider to control vertical rotation
	final JSlider pitchSlider = new JSlider(SwingConstants.VERTICAL, -90, 90, 0);
	//int outWidth = 1280;
	//int outHeight = 1000;
	int outWidth = 640;
	int outHeight = 480;
    double[] zBuffer = new double[outWidth*outHeight];
    
    ByteBuffer cbL, cbR;
    byte[] bufferL = new byte[0];
    byte[] bufferR = new byte[0];
   
	String mode = "";
	String outDir = "/users/jg/workspace/robocore/motionclouds";
	int frames = 0;
	int files = 0;
	int[] ibuf = null;
	// optical parameters of Logitech C310 and others

    final static float f = 4.4f; // focal length mm
    final static float B = 205.0f; // baseline mm
    final static float FOVD = 60; // degrees field of view
    final static double FOV = 1.04719755; // radians FOV
    final static int camWidth = 640; // pixels
    final static int camHeight = 480;
    final static double fp = B*(camWidth*.5)/Math.tan(FOV *.5 * (Math.PI/2)); //focal length in pixels
    final static double Bf = B*f;// calc the Bf of Bf/d
    // block matching constants
    final static int maxHorizontalSep = (int) (Bf-1)/4; // max pixel disparity
    final static int corrWinSize = 15; // Size of eventual depth patch
    final static int corrWinLeft = 7; // number of left/up elements in corr window
    final static int corrWinRight = 8;// number of right/down elements in corr window

    CircularBlockingDeque<BufferedImage> queueL = new CircularBlockingDeque<BufferedImage>(10);
    CircularBlockingDeque<BufferedImage> queueR = new CircularBlockingDeque<BufferedImage>(10);
    
	CyclicBarrier latchTrans = new CyclicBarrier(camHeight/corrWinSize);
	
    //static int threads = 0;

    byte[] bqueue;
	private int sequenceNumber,lastSequenceNumber;
	long time1;

	Object mutex = new Object();
	FloatDCT_2D fdct2dL = new FloatDCT_2D(corrWinSize, corrWinSize);
	FloatDCT_2D fdct2dR = new FloatDCT_2D(corrWinSize, corrWinSize);

    CannyEdgeDetector ced = null;
    
	BufferedImage bimage = null;
	short[][][] simage = null; // [x][y]([r][g][b][d])
	//CyclicBarrier latch = new CyclicBarrier(camHeight/corrWinSize+1);
	//CyclicBarrier latchOut = new CyclicBarrier(camHeight/corrWinSize+1);
	CyclicBarrier latch = new CyclicBarrier(2);
	CyclicBarrier latchOut = new CyclicBarrier(2);
	Matrix3 transform;
	//int yStart;
	int threads = 0;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("subs_videoproc");
	}
	
	@Override
	public void onStart(final ConnectedNode connectedNode) {

		Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
		if( remaps.containsKey("__mode") )
			mode = remaps.get("__mode");
		if( mode.startsWith("display") || mode.equals("hough")) {
			if( DEBUG )
				System.out.println("Pumping frames to AWT Panel");
			SwingUtilities.invokeLater(new Runnable() {
			    public void run() {
			        displayPanel = new PlayerFrame();
			    }
			});
		}
		// construct gaussian kernel with peak at 127
		//final int[][] kernel = ImgProcessor.gaussianNormal(127,kernWinSize);

		/**
		 * Thread to precompute the correspondence windows for each pixel in a new image and store those
		 * to the array backed blocking queue for retrieval by the main processing thread.
		 * This thread works strictly on left source image, while the main thread processes the right image with
		 * the kernels generated here and stored on the queue.
		ThreadPoolManager.getInstance().spin(new Runnable() {
				@Override
				public void run() {
					int[][] imgsrc = new int[corrWinSize][corrWinSize]; // left image subject pixel and surrounding
			        while(true) {
			        	synchronized(mutex) {
			        		try {
								mutex.wait();
							} catch (InterruptedException e) {}
			        	}
			        	corrWins.clear();
		        		
		        		// xsrc is index for subject pixel
		        		// correlation window assumed symmetric left/right/up/down with odd number of elements
		        		for(int y = corrWinLeft; y < imageL.getHeight()-corrWinRight; y++) {
		        			// for each subject pixel X in left image, sweep the scanline for that Y, move to next subject X
		        			for(int xsrc = corrWinLeft; xsrc < imageL.getWidth()-corrWinRight; xsrc++) {
		        				// check if main thread signaled we found a perfect score
		        				if(sadZero.get() != 0) {
		        					corrWins.clear();
		        					if( sadZero.get() != y)
		        						System.out.println("**POSSIBLE MISMATCH WITH WINDOW GENERATOR**");
		        					sadZero.set(0);
		        					break; // go to next y
		        				}
		        				// sweep the epipolar scan line for each subject pixel in left image
		        				// first create the subject pixel kernel and weight it
		        				int kx = 0;
		        				for(int i = xsrc-corrWinLeft; i < xsrc+corrWinRight; i++) {
		        						int ky = 0;
		        						for(int j = y-corrWinLeft; j < y+corrWinRight; j++) {
		        							// weight the high 8 bits with our gaussian distro values which have 127 max so sign should be unaffected
		        							imgsrc[kx][ky] = (imageL.getRGB(i,j) & 0xFFFFFF) | (kernel[kx][ky] << 24);
		        							++ky;
		        						}
		        						++kx;
		        				}
		        				corrWins.add(imgsrc);
		        			}
		        		}
			        }
				}
		}, "SYSTEM");
		*/
		/*
		 * Main worker thread for image data. 
		 * Wait at synch barrier for completion of all processing threads, then display disparity 
		 */
		final AtomicInteger yStart = new AtomicInteger(0);
		ThreadPoolManager.getInstance().spin(new Runnable() {
				@Override
				public void run() {
					System.out.println("Correspondence window size="+corrWinSize+" processing="+(camHeight-corrWinSize));
					while(true) {
					//
					// Spin the threads for each chunk of image
					//
					try {
					  latch.await();
					  yStart.set(0);
					  for(int xStart = 0; xStart < camHeight-corrWinSize; xStart++) {
						//for(; threads < camHeight/corrWinSize; threads++) {
						//System.out.println("Spinning thread "+yStart);
						//ThreadPoolManager.getInstance().spin(new Runnable() {
						FixedThreadPoolManager.getInstance(camHeight/corrWinSize).spin(new Runnable() {
						  //int yStart = threads*corrWinSize;
						  //int yEnd = yStart+corrWinSize-1;
						  @Override
						  public void run() {
							//try {
									//while(true) {
										//try {
											//latch.await();
											// Since we run a continuous loop inside run we have to have a way
											// to synchronize everything at the beginning of a new image, then
											// await completion of all processing threads and signal a reset to
											// resume processing at the start of another new image, hence the two barrier
											// synchronization latches
											switch(mode) {
												// Display results as greyscale bands of depth overlayed on image in a java window
												case "display":
													processImageChunk(imageL, imageR, yStart.getAndIncrement(), camWidth, bimage);
													break;
												// Display a 3Dish rendering of image with depth values and sliders for orientation
												case "display3D":
													processImageChunk3D(imageL, imageR, yStart.getAndIncrement(), camWidth, bimage);
													break;
												// Perform canny edge detect then depth value those and overlay, writing CloudCompare file
												case "edge":
													processImageChunkEdge(imageL, imageR, imageLx, imageRx, yStart.getAndIncrement(), camWidth, simage);
													break;
												// Perform depth then deliver array of values
												default:
													//processImageChunk(imageL, imageR, imageT, yStart, camWidth, transform, true, simage);
													processImageChunk(imageL, imageR, imageT, yStart.getAndIncrement(), camWidth, null, true, simage);
											}
											//latchOut.await();
										//} catch (BrokenBarrierException e) { System.out.println("<<BARRIER BREAK>> "+this);}
									//}
							//} catch (InterruptedException e) { e.printStackTrace(); }
						  } // run
					    }, "SYSTEMFIX");
					  //}, "SYSTEM");
					  } // for yStart
					  BlockingQueue<Runnable> bq = FixedThreadPoolManager.getInstance(camHeight-corrWinSize).getQueue();
					  while(!bq.isEmpty()) Thread.sleep(1);
					  latchOut.await();
					} catch (BrokenBarrierException | InterruptedException e) { System.out.println("<<BARRIER BREAK>> "+this);}
					} // while true
			} // run
			//} catch (BrokenBarrierException | InterruptedException e) { System.out.println("<<BARRIER BREAK>> "+this);}
		}, "SYSTEM");
					
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
			        			latchOut.reset();
			        			latch.reset();
								imageLx = queueL.takeFirst();
				        		imageRx = queueR.takeFirst();
				        		//
				        		switch(mode) {
				        		case "display3D":
				        			imageL = imageLx;
				        			imageR = imageRx;
				        			//score = new int[imageL.getWidth()];
				        			if( bimage == null ) {
				        				bimage = new BufferedImage(outWidth, outHeight, BufferedImage.TYPE_INT_ARGB);
				        				//bimage = new BufferedImage(outWidth, outHeight, BufferedImage.TYPE_INT_RGB);
				        				synchronized(bimage) {
			        						Graphics g = bimage.getGraphics();
			        						g.setColor(Color.WHITE);
			        						g.fillRect(0, 0, outWidth, outHeight);
			        					}
				        			}
			        				//System.out.println("setting display "+bimage);
			        				latch.await();
			        	     		latchOut.await();
			        				displayPanel.setLastFrame(bimage);
			        				displayPanel.invalidate();
			        				displayPanel.updateUI();
			        				if( viewChanged) {
			        					viewChanged = false;
			        				//	synchronized(bimage) {
			        				//		Graphics g = bimage.getGraphics();
			        				//		g.setColor(Color.WHITE);
			        				//		g.fillRect(0, 0, outWidth, outHeight);
			        				//	}
			        				}
			        				break;
				        		case "display":
				        			imageL = imageLx;
				        			imageR = imageRx;
				        			if( bimage == null ) {
				        				bimage = new BufferedImage(outWidth, outHeight, BufferedImage.TYPE_INT_ARGB);
				        			}
			        				//System.out.println("setting display "+bimage);
			        				latch.await();
			        	     		latchOut.await();
			        				displayPanel.setLastFrame(bimage);
			        				displayPanel.invalidate();
			        				displayPanel.updateUI();
			        				break;
				        		case "hough":
					        			imageL = imageLx;
					        			imageR = imageRx;
				        				if( bimage == null )
					        				bimage = new BufferedImage(outWidth, outHeight, BufferedImage.TYPE_INT_ARGB);
				        				if( simage == null)
				        					simage = new short[outWidth][outHeight][4];
				        				latch.await();
				        				latchOut.await();
				        				processHough(bimage, simage, outWidth, outHeight);
				        				displayPanel.setLastFrame(bimage);
				        				displayPanel.invalidate();
				        				displayPanel.updateUI();
				        				break;
				        		case "edge":		
					        	     	synchronized(imageLx) {
					        	     		ced = new CannyEdgeDetector();
					        				ced.setSourceImage(imageLx);
					            			ced.process();
					            			imageL = ced.getEdgesImage();
					        	     	}
					        	     	synchronized(imageRx) {
					        	     		ced = new CannyEdgeDetector();
					        				ced.setSourceImage(imageRx);
					            			ced.process();
					            			imageR = ced.getEdgesImage();
					        	     	}
					        	     		
				        				imageL = imageLx;
				        				imageR = imageRx;	
				        				if( simage == null)
				        					simage = new short[outWidth][outHeight][4];
				        		   		// write source images
				        	     		synchronized(imageL) {
				        	     			writeFile("sourceL", imageL, ++files);
				        	     		}
				        	     		synchronized(imageR) {
				        	       			writeFile("sourceR", imageR, files);
				        	     		}
				        				latch.await();
				        	     		latchOut.await();
				        	     		synchronized(simage) {
				        	     			writeFile(simage);
				        	     		}
				        	     		break;
				        	     	default:
				        				imageL = imageLx;
				        				imageR = imageRx;
				        				/*
				        				transform = new Matrix3(new double[] /*{0.99939083, 0.01020363, -0.03337455,
				        						0.00000000, 0.95630476, 0.29237170,
				        						0.03489950, -0.29219360,0.95572220});
				        							{0.99984770,    -0.00091339,    -0.01742849,
				        						    0.00000000,     0.99862953,    -0.05233596,
				        						    0.01745241,     0.05232799,     0.99847744});
				        				if( imageT == null )
					        				imageT = new BufferedImage(outWidth, outHeight, BufferedImage.TYPE_INT_RGB);
				        				*/
				        				if( simage == null)
				        					simage = new short[outWidth][outHeight][4];
				        		   		// write source images
				        	     		synchronized(imageL) {
				        	     			writeFile("sourceL", imageL, ++files);
				        	     		}
				        	     		synchronized(imageR) {
				        	       			writeFile("sourceR", imageR, files);
				        	     		}
				        	     		//latchTrans.reset();
				        				latch.await();
				        	     		latchOut.await();
				        	     		synchronized(simage) {
				        	     			writeFile(simage);
				        	     		}
				        	     		/*
				        	     		synchronized(imageT) {
				        	     			writeFile("sourceT", imageT, files);
				        	     		}
				        	     		*/
				        	     		break;
				        				//navigate(simage, 213, 426);
				        			
				        		}
				        		// tell the image processing threads that a new image is ready
				        		//latch.await();
				        		// now wait for the threads to finish processing
				        		//latchOut.await();
							} catch (InterruptedException | BrokenBarrierException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
							}
			        		//imageL = null;
			        		//imageR = null;
			        	}
			        } // while true
				} // run      
		}, "SYSTEM");
		
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
			//image = new BufferedImage(img.getWidth(), img.getHeight(), BufferedImage.TYPE_3BYTE_BGR);
			//image = new BufferedImage(img.getWidth(), img.getHeight(), BufferedImage.TYPE_4BYTE_ABGR);
			/*
			image = new BufferedImage(img.getWidth(), img.getHeight(), BufferedImage.TYPE_INT_ARGB);
			WritableRaster raster = (WritableRaster) image.getRaster();
			int ibsize = img.getHeight() * img.getWidth();
			if( ibuf == null )
				ibuf = new int[ibsize * 4];
			int iup = 0;
			int ip = 0;
			for(int i = 0; i < ibsize; i++) {
				int ialph = 255; //(ib.get(i) >> 24) & 0x0ff;
				//int ired = (ib.get(i) >> 16) & 0x0ff; 
				//int igreen = (ib.get(i) >> 8 ) & 0x0ff;
				//int iblue = (ib.get(i) & 0x0ff);
				int iblue = bufferL[ip++];
				int igreen = bufferL[ip++];
				int ired = bufferL[ip++];
				ibuf[iup++] = ired;
				ibuf[iup++] = igreen;
				ibuf[iup++] = iblue;
				ibuf[iup++] = ialph;
			}
			//System.out.println(ibuf.length+" "+raster.getWidth()+" "+raster.getHeight()+" "+raster.getMinX()+" "+raster.getMinY());
		    raster.setPixels(0, 0, img.getWidth(), img.getHeight(), ibuf);
			*/
	
			} catch (IOException e1) {
				System.out.println("Could not convert image payload due to:"+e1.getMessage());
				return;
			}
				//displayPanel.setLastFrame((java.awt.Image)image);
				//displayPanel.setLastFrame(displayPanel.createImage(new MemoryImageSource(newImage.imageWidth
				//		, newImage.imageHeight, bufferL, 0, newImage.imageWidth)));
				//displayPanel.invalidate();
				//displayPanel.updateUI();
				//queue.addLast(image);
			++sequenceNumber; // we want to inc seq regardless to see how many we drop	
		}
	  });
		
	} // onstart
	
	/**
	 * Write CloudCompare point cloud viewer compatible file
	 * for each point - X,Y,Z,R,G,B ascii delimited by space
	 * @param simage the processed array chunks of [x][y][0]=R, [x][y][1]=G, [x][y][2]=B, [x][y][3]=D
	 */
	protected void writeFile(short[][][] simage) {
		DataOutputStream dos = null;
		File f = new File(outDir+"/roscoe"+(++frames)+".asc");
		try {
			dos = new DataOutputStream(new FileOutputStream(f));
			for(int y = 0; y < outHeight; y++) {
				for(int x = 0; x < outWidth; x++) {
					dos.writeBytes(String.valueOf(x));
					dos.writeByte(' ');
					dos.writeBytes(String.valueOf(y));
					dos.writeByte(' ');
					dos.writeBytes(String.valueOf(simage[x][y][3])); // Z
					dos.writeByte(' ');
					dos.writeBytes(String.valueOf(simage[x][y][0]));
					dos.writeByte(' ');
					dos.writeBytes(String.valueOf(simage[x][y][1]));
					dos.writeByte(' ');
					dos.writeBytes(String.valueOf(simage[x][y][2]));
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
	 * Process the stereo images into a 3D rendered image, orientation is based on horizontal and vertical sliders
	 * in the panel. The horizontal slider is heading, vertical slider is pitch.
	 * In practice the processing threads will segment the image into several regions to be processed in parallel
	 * differentiated by yStart and yEnd.
	 * @param imageL Left image from bus
	 * @param imageR Right image from bus
	 * @param yStart Starting Y band to process
	 * @param width width of image band
	 * @param simage buffered image to hold 3D rendering
	 * @param kernel Gaussian weighted kernel for correlation
	 */
	public void processImageChunk3D(BufferedImage imageL, BufferedImage imageR, int yStart, int width, BufferedImage bimage) {
			double heading = Math.toRadians(headingSlider.getValue());
   			Matrix3 headingTransform = new Matrix3(new double[] {
                    Math.cos(heading), 0, -Math.sin(heading),
                    0, 1, 0,
                    Math.sin(heading), 0, Math.cos(heading)
            });
   			double pitch = Math.toRadians(pitchSlider.getValue());
   			Matrix3 pitchTransform = new Matrix3(new double[] {
                    1, 0, 0,
                    0, Math.cos(pitch), Math.sin(pitch),
                    0, -Math.sin(pitch), Math.cos(pitch)
            });
   			Matrix3 transform = headingTransform.multiply(pitchTransform);
   			if( viewChanged) {
   				System.out.println(Thread.currentThread().getName()+" "+yStart+","+(yStart+corrWinSize )+"\r\n"+transform);
   			}
			
			// initialize array with extremely far away depths
			//for (int q = 0; q < zBuffer.length; q++) {
			//	zBuffer[q] = Double.NEGATIVE_INFINITY;
			//}
    		// SAD - sum of absolute differences. take the diff of RGB values of
    		// win 1 and 2 and sum them
    		//int[][] imgsrc = new int[corrWinSize][corrWinSize]; // left image subject pixel and surrounding
    		int[] imgsrcL = new int[width*corrWinSize]; // left image scan line and corr window
     		int[] imgsrcR = new int[width*corrWinSize]; // right image scan line and corr window
    		// xsrc is index for subject pixel
    		// correlation window assumed symmetric left/right/up/down with odd number of elements
    		// so we use corrWinLeft as top half of array size and corrwinRight as bottom half
     		//int close = 0;
     		//int far = 0;
    		//for(int y = yStart; y < yEnd; y+=corrWinSize/*y++*/) {
    			synchronized(imageL) {
    				imageL.getRGB(0, yStart, width, corrWinSize, imgsrcL, 0 , width);
    			}
    			synchronized(imageR) {
    				imageR.getRGB(0, yStart, width, corrWinSize, imgsrcR, 0 , width);
    			}
    			// for each subject pixel X in left image, sweep the scanline for that Y, move to next subject X
    			loop:
    			for(int xsrc = 0; xsrc < width; xsrc++) {
    				// sweep the epipolar scan line for each subject pixel in left image
    				// first create the subject pixel kernel and weight it
    				//int kx = 0;
    				/*
    				for(int i = xsrc-corrWinLeft; i < xsrc+corrWinRight; i++) {
    						int ky = 0;
    						for(int j = y-corrWinLeft; j < y+corrWinRight; j++) {
    							// weight the high 8 bits with our gaussian distro values which have 127 max so sign should be unaffected
    							//imgsrc[kx][ky] = (imageL.getRGB(i,j) & 0xFFFFFF) | (kernel[kx][ky] << 24);
    							imgsrcL[ky*width+i] = (imgsrcL[ky*width+i] & 0xFFFFFF) | (kernel[kx][ky] << 24);
    							++ky;
    						}
    					++kx;
    				}
    				*/
					//int rank = 0;
					//int pix = 0;
					//int winL = xsrc-corrWinLeft;
    				// now begin sweep of scanline at y beginning at 1 plus subject x
    				//for(int x = xsrc+1; x < width-corrWinRight; x++) {
    						// imgsrc at kx,ky is target in weighted left correlation window
    						//kx = 0;
    						//int sum = 0;
    						// outer loop sweeps x in right image starting at subject left image x+1 - left half of
    						// correlation window size, to x+1 + right half of correlation window size
    						//for(int i = x-corrWinLeft; i < x+corrWinRight; i++) {
    							//int ky = 0;
    							// recall; symmetric left/right up down
    							// search at current pixel offset left, to current pixel + offset right, applied to y
    							//for(int j = y-corrWinLeft; j < y+corrWinRight; j++) {
    								//int rrgb = imageR.getRGB(i, j);
    								//int rrgb = imgsrcR[ky*width+i];
    								//rrgb &= 0xFFFFFF;
    								// sum of absolute difference of left image weight+RGB and right image weight+RGB
    								//sum += Math.abs(imgsrc[kx][ky] - (rrgb | (kernel[kx][ky] << 24)));
    								//sum += Math.abs(imgsrcL[ky*width+(winL+kx)] - (rrgb /*|(kernel[kx][ky] << 24)*/));
    								//++ky;
    							//}
    							// increment x of left source weighted correlation window
    							//++kx;
    						//}
    						// score array is sized to image width and we ignore irrelevant edge elements
    						//score[x] = sum;
    						//if( sum <= 127) {
    							//++close;
    							// set sadZero to current y value to signal kernel generator to progress to next y
    							//sadZero.set(y);
    							//rank = x;
		        				//pix = (int)(Bf/Math.abs(xsrc-rank));
		        				//if( pix <=0 || pix >=maxHorizontalSep)
		        					//System.out.print(xsrc+","+y+" p="+pix+"|");
		        				//bimage.setRGB(xsrc, y, (byte)pix); //greyscale option, not so hot
		        				//for( int k = xsrc-corrWinLeft; k < xsrc+corrWinRight; k++) {
		        					//int m = y - corrWinLeft;
		        					for(int l = 0; l < corrWinSize; l++) {		
		        					//for(int l = y-corrWinLeft; l < y+corrWinRight; l++) {
		        						//bimage.setRGB(k, l, ((imageL.getRGB(k,l) & 0xFFFFFF) | pix<<24));
		        						// replace weighted gaussian distro value with computed disparity val and 3D xform
		        						// z depth negative since positive is toward viewer
		        						int ks, ms;
		        						int[] c = scale(xsrc,l+yStart, transform);
		        						ks = c[0];
		        						ms = c[1];
		        						//Vertex v0 = depthToVertex(k, m+l, pix, camWidth, camHeight, FOV);
		        						//Point t = new Point(v0, new Color(((imgsrcL[l*width+k] & 0xFFFFFF))));
		               					//Vertex v1 = transform.transform(t);
		               					//Vertex norm = new Vertex(v1.x,v1.y,v1.z);       
		               					//double normalLength = Math.sqrt(norm.x * norm.x + norm.y * norm.y + norm.z * norm.z);
		               					//norm.x /= normalLength;
		               					//norm.y /= normalLength;
		               					//norm.z /= normalLength;
		               					//double angleCos = Math.abs(norm.z);
		               					//int zIndex = (int)v1.y * outWidth + (int)v1.x;
		               					//System.out.println(v1.x+","+v1.y+","+v1.z+","+zIndex+" anglecos="+angleCos);
		               					
		               					
		        						// ks and ms should now be transformed to coord of right image that slides to left
		        						
		               					//if( zIndex < zBuffer.length && zIndex >= 0) {
		               					//	synchronized(zBuffer) {
		               					//		if (zBuffer[zIndex] < v1.z) {
		               					//			boolean oob = false;
		        									int lgbr = 0xFFFF0000;
		        									synchronized(imageL) {
		        										try {
		        											lgbr = imageL.getRGB(ks, ms);
		        										} catch(ArrayIndexOutOfBoundsException aioob) {}
		        									}
		               								synchronized(bimage) {
		               									try {	
		               										//bimage.setRGB((int)v1.x, (int)v1.y, getShade(t.color, angleCos).getRGB());
		               										// original right target pixel, translated up and over, xor with left subject pixel
		               										bimage.setRGB(ks, ms, lgbr ^ (imgsrcR[l*width+xsrc] & 0xFFFFFF));
		               									} catch(ArrayIndexOutOfBoundsException aioob) {
		               										System.out.println("Coord out of bounds:"+ks+","+ms+" "+Thread.currentThread().getName());
		               										bimage.setRGB(xsrc, yStart+l, 0xFFFF0000); // make red
		               					//					oob = true;
		               									}
		               								}
		               					//			if( oob )
		               					//				System.out.println("Coord out of bounds:"+v1+" point:"+t+" orig:"+k+","+(m+l)+","+pix);
		               					//			zBuffer[zIndex] = v1.z;
		               					//		}
		               					//	}
		               					//} else {
		               					//	System.out.println("Zindex out of bounds="+zIndex+" "+v1+" point:"+t);
		               					//}
		               					
		        						//imgsrcL[l*width+k] = ((imgsrcL[l*width+k] & 0xFFFFFF) | pix<<24);
		               					//++m;
		        					//}
		        				//}
		        				// move to next x at outer x scanline loop
    							//continue loop;
    						}
    							
    				}
    				// now calculate the one closest to zero from sum of differences
    				// rank = 0
    				//++far;
    				//int drank = Integer.MAX_VALUE;
    				//for(int s = xsrc+1; s < width-corrWinRight; s++) {
    					//System.out.print("Y="+y+" score "+s+"="+score[s]+" ");
    					//if (score[s] < drank) {
    						//rank = s;
    						//drank = score[s];
    					//} 
						/*else {
    						if (score[s] == drank && score[s] > 0 && score[rank] < 0) {
    							//same distance to zero but positive 
    							rank = s;
    						}
    						
    					}*/
    				//}
    				//calc the disparity and insert into disparity map image
    				//pix = (int)(Bf/Math.abs(xsrc-rank));
    				//if( pix <=0 || pix >=maxHorizontalSep)
    				//	System.out.print(xsrc+","+y+" p="+pix+"|");
    				//for( int k = xsrc-corrWinLeft; k < xsrc+corrWinRight; k++) {
    					//int m = y - corrWinLeft;
    					//for(int l = y-corrWinLeft; l < y+corrWinRight; l++) {
    					//for(int l = 0; l < corrWinSize; l++) {
    						//bimage.setRGB(k, l, ((imageL.getRGB(k,l) & 0xFFFFFF) | pix<<24));
    						// replace weighted gaussian distro value with computed disparity val and 3D xform
    						// x is the same axis, the y axis is depth, and z is original Y
    						//Point t = new Point((double)k, (double)m, (double)pix, new Color(((imgsrcL[l*width+k] & 0xFFFFFF))));
    						//Vertex v0 = depthToVertex(k, m+l, pix, camWidth, camHeight, FOV);
    						//Point t = new Point(v0, new Color(((imgsrcL[l*width+k] & 0xFFFFFF))));
    						// z depth made negtive because positive is toward viewer
    						//int ks, ms;
    						//if( k <= camWidth/2 )
    						//	ks = (camWidth/2) - k;
    						//else
    						//	ks = k - (camWidth/2);
    						//if( (m+l) <= camHeight/2 )
    						//	ms = (camHeight/2) - (m+l);
    						//else
    						//	ms = (m+l) - (camHeight/2);
    						//Point t = new Point(ks,ms, -pix, new Color(((imgsrcL[l*width+k] & 0xFFFFFF))));
           					//Vertex v1 = transform.transform(t);
           					//Vertex norm = new Vertex(v1.x,v1.y,v1.z);       
           					//double normalLength = Math.sqrt(norm.x * norm.x + norm.y * norm.y + norm.z * norm.z);
           					//norm.x /= normalLength;
           					//norm.y /= normalLength;
           					//norm.z /= normalLength;
           					//double angleCos = Math.abs(norm.z);
           					//int zIndex = (int)v1.y * outWidth + (int)v1.x;
           					//System.out.println(v1.x+","+v1.y+","+v1.z+","+zIndex+" anglecos="+angleCos);
           					
           					/*if( zIndex < zBuffer.length && zIndex >= 0) {
           						synchronized(zBuffer) {
           							if (zBuffer[zIndex] < v1.z) {*/
           								//boolean oob = false;
           								//synchronized(bimage) {
           									//try {
           										//bimage.setRGB((int)v1.x, (int)v1.y, getShade(t.color, angleCos).getRGB());
           										//bimage.setRGB((int)v1.x, (int)v1.y, t.color.getRGB());
           									//} catch(ArrayIndexOutOfBoundsException aioob) {
           									//	oob = true;
           									//}
           								//}
           								//if( oob )
           								//	System.out.println("Coord out of bounds:"+v1+" point:"+t+" orig:"+k+","+(m+l)+","+pix);
           								/*zBuffer[zIndex] = v1.z;
           							} 
           						}
           						
       						} else {
       							System.out.println("Zindex out of bounds="+zIndex+" "+v1+" point:"+t);
       						}
       						*/
       						
           					//++m;
    						// replace weighted gaussian distro value with computed disparity val in alpha channel
    						//imgsrcL[l*width+k] = ((imgsrcL[l*width+k] & 0xFFFFFF) | pix<<24);
    					//}
    				//}
    				//bimage.setRGB(xsrc, y, (byte)pix);
    				//bimage.setRGB(xsrc, y, ((imageL.getRGB(xsrc,y) & 0xFFFFFF) | pix<<24));
    			//} //++xsrc  move to next x subject in left image at this scanline y
    			//System.out.println("Y scan="+y);
    		//} // next y
    		//if( SAMPLERATE )
    		//	System.out.println("**********END OF 3D IMAGE "+yStart+","+yEnd/*+" close="+close+" far="+far+" ***********"*/);
	}
	/**
	 * Process the stereo images into a new BufferedImage with simulated greyscale representing depth
	 * and uncorrelated patches colored red 
	 * In practice the processing threads will segment the image into several regions to be processed in parallel
	 * differentiated by yStart and yEnd.
	 * THIS METHOD DESIGNED TO BE USED IN A SINGLE THREAD PER IMAGE BAND, PROCESSING corrWinSize chunks.
	 * @param imageL Left image from bus
	 * @param imageR Right image from bus
	 * @param yStart Starting Y band to process
	 * @param width width of image band
	 * @param bimage buffered image to hold result
	 */
	public void processImageChunk(BufferedImage imageL, BufferedImage imageR, int yStart, int width, BufferedImage bimage) {
			long etime = System.currentTimeMillis();
    		// SAD - sum of absolute differences. take the diff of RGB values of
    		// win 1 and 2 and sum them	
			int[] score = new int[width];
    		int[] imgsrcL = new int[width*corrWinSize]; // left image scan line and kernel window
     		int[] imgsrcR = new int[width*corrWinSize]; // right image scan line and kernel window
     		float[] coeffsL = new float[corrWinSize*corrWinSize];
     		// precompute right side DCT values since we will be sweeping over them repeatedly as we march forward in left X
     		float[][] coeffsR = new float[width][corrWinSize*corrWinSize];
    		// xsrc is index for subject pixel
    		// correlation window assumed symmetric left/right/up/down with odd number of elements
    		// so we use corrWinLeft as top half of window array size and corrwinRight as bottom half
     		int close = 0;
     		int far = 0;
     		int xminL = 0;
     		int xmaxL = 0;
     		int xminR = 0;
     		int xmaxR = 0;
     		int trunc = 0;
     		
 			synchronized(imageL) {
				imageL.getRGB(0, yStart, width, corrWinSize, imgsrcL, 0 , width);
			}
			synchronized(imageR) {
				imageR.getRGB(0, yStart, width, corrWinSize, imgsrcR, 0 , width);
			}

			//
			// Precompute DCTs in right image.
			// 
			for(int x = 0; x < width; x++) {
	  			xminR = x;
	  			xmaxR = x+corrWinSize;
   				if(  x > width-corrWinSize) {
   					xmaxR = width;
   					xminR = width-corrWinSize;
				}
				// loop sweeps x in right image, we increment by one instead of corrWinSize as we do in outer loop
				// because we need to find every potential match. 
				//System.out.println("X scan="+x+" winsize="+winSize+" xCorrWinL="+xCorrWinL+" xCoorWinR="+xCorrWinR+"@"+Thread.currentThread().getName());
					//
				int coeffsRi = 0;
				//
				// j increments in y for right image
				for(int j = 0; j < corrWinSize; j++) {
					// recall; symmetric left/right up down
					// search at current pixel offset left, to current pixel + offset right, applied to y
					// i increments in x, row major order
					for(int i = xminR; i < xmaxR; i++) {
						int rrgb = imgsrcR[j*width+i];
						//
						rrgb &= 0xFFFFFF;
						//coeffsR[coeffsRi++] = rrgb;
						int rr = (rrgb & 0xFF0000) >> 24;
						int rg = (rrgb & 0x00FF00) >> 16;
						int rb = (rrgb & 0x0000FF);
						int pixR = rr + rg + rb;
						//int pixR = (int) (((float)rr *.299) + ((float)rg *.587) + ((float)rb *.114));
						coeffsR[x][coeffsRi++] = pixR;
						   // Normalize and gamma correct:
					        //double rfrr = Math.pow(rr / 255.0, 2.2);
					        //double rfgg = Math.pow(rg / 255.0, 2.2);
					        //double rfbb = Math.pow(rb / 255.0, 2.2);
					        // Calculate luminance:
					        //double rflum = 0.2126 * rfrr + 0.7152 * rfgg + 0.0722 * rfbb;
					        // Gamma compand and rescale to byte range:
					        //int pixR = (int) (255.0 * Math.pow(rflum, 1.0 / 2.2));
							// sum of absolute difference of left image weight+RGB and right image weight+RGB
							//sum += Math.abs((pixL /*| (kernel[kx][ky] << 16)*/) - (pixR /*| (kernel[kx][ky] << 16)*/));
					}
				}
				fdct2dR.forward(coeffsR[x], false);
			}
  			// for each subject pixel X in left image, sweep the scanline for that Y, move to next subject X
			// We always process in corrWinSize chunks
			loop:
			//for(int xsrc = corrWinLeft; xsrc < width-corrWinRight; /*xsrc+=corrWinSize*/xsrc++) {
	  		for(int xsrc = 0; xsrc < width; xsrc++) {	
				// sweep the epipolar scan line for each subject pixel in left image
  	  			xminL = xsrc;
	  			xmaxL = xsrc+corrWinSize;
   				if(  xsrc > width-corrWinSize) {
   					xmaxL = width;
   					xminL = width-corrWinSize;
				}
				int rank = 0;
				int pix = 0;
			    // process left image data, fill IDCT array row major order.
				// Remember, we have extracted the corrWinSize*width band from image, and now deal with that
				int coeffsLi = 0; // DCT float array counter left
				for(int ly = 0; ly < corrWinSize; ly++) {
					for(int lx = xminL; lx < xmaxL; lx++) {
						int lrgb = imgsrcL[ly*width+lx];
						//lrgb &= 0xFFFFFF;
						//coeffsL[coeffsLi++] = lrgb;
						int lr = (lrgb & 0xFF0000) >> 24;
						int lg = (lrgb & 0x00FF00) >> 16;
						int lb = (lrgb & 0x0000FF);
						//System.out.println(lr+" "+lg+" "+lb);
						int pixL = lr + lg + lb; // greyscale
						//int pixL = (int) (((float)lr *.299) + ((float)lg *.587) + ((float)lb *.114)); // human greyscale
						coeffsL[coeffsLi++] = pixL;
					}
				}
				// try DCT
				//fdct2dL.inverse(coeffsL, false);
				fdct2dL.forward(coeffsL, false);
				//
				// Left image window set up, now begin sweep of scanline at y in right image.
				// variable x tracks the right image x position
				// 
				int sum = 0;
    			for(int x = 0; x < width; x++) {
    				// skip the subject
    				//if( x == xsrc ) {
    				// skip the subject and if the right chunk starting at x is devoid of edge, skip as well
    				if( x == xsrc ) {
    					score[x] = Integer.MAX_VALUE;
    					continue;
    				}
    	  			xminR = x;
    	  			xmaxR = x+corrWinSize;
       				if(  x > width-corrWinSize) {
	   					xmaxR = width;
	   					xminR = width-corrWinSize;
					}
					// loop sweeps x in right image, we increment by one
					// because we need to find every potential match. 
					//System.out.println("X scan="+x+" winsize="+winSize+" xCorrWinL="+xCorrWinL+" xCoorWinR="+xCorrWinR+"@"+Thread.currentThread().getName());
					// Compute the sum of absolute difference SAD between the 2 matrixes representing
					// the left subject subset and right target subset
					sum = 0;
					for(int isum = 0; isum < corrWinSize*corrWinSize; isum++) {
						//sum += Math.abs(coeffsL[isum]-coeffsR[isum]);
						sum += Math.abs(coeffsL[isum]-coeffsR[x][isum]);
						// sum of squared diffs
						//sum += Math.pow((coeffsL[isum]-coeffsR[isum]),2);
					}
					// score array is sized to image width and we ignore irrelevant edge elements
					score[x] = sum;
 					// If we have a value thats as small as possible, we can assume the differences are zero and this is our target
					// This is just an optimization to save time computing smallest element
					if( sum == 0) {
						++close;
						// set sadZero to current y value to signal kernel generator to progress to next y
						//sadZero.set(y);
						rank = x;
	        			pix = (int) (Bf/Math.abs(xsrc-rank));
		        		//System.out.print(xsrc+","+y+" p="+pix+"|");
		        		if( pix > 255) {
		        			//	System.out.println("CLOSE PIX TRUNCATED FROM:"+xsrc+","+y+" p="+pix);
		        			++trunc;
		        			pix = 255;
		        			// make the vertical band red, indicating its been truncated
		            		for(int l = 0; l < corrWinSize; l++) {
		            			imgsrcL[l*width+xsrc] = ((imgsrcL[l*width+xsrc] & 0x00FFFFFF) | 255<<24);
		            			imgsrcL[l*width+xsrc] = ((imgsrcL[l*width+xsrc] & 0xFF000000) | 255<<16);
		            		}
		        		} else {
		        			//bimage.setRGB(xsrc, y, (byte)pix); //pixel by pixel ersatz greyscale option, not so hot
		        			// bulk pixel scanline setup
		        			for(int l = 0; l < corrWinSize; l++) {
		        			//for(int l = y-corrWinLeft; l < y+corrWinRight; l++) {
		        				//bimage.setRGB(k, l, ((imageL.getRGB(k,l) & 0xFFFFFF) | pix<<24));
		        				// replace weighted gaussian distro value with computed disparity val in alpha channel
		        				//imgsrcL[l*width+k] = ((imgsrcL[l*width+k] & 0xFFFFFF) | pix<<24);
		        				// try a simulated greyscale
		        				int pixG = (int) (((float)pix *.299) + ((float)pix *.587) + ((float)pix *.114));
		        				imgsrcL[l*width+xsrc] = ((imgsrcL[l*width+xsrc] & 0x00FFFFFF) | 127<<24);
		        				imgsrcL[l*width+xsrc] = ((imgsrcL[l*width+xsrc] & 0xFF00FFFF) | pixG<<16);
		        				imgsrcL[l*width+xsrc] = ((imgsrcL[l*width+xsrc] & 0xFFFF00FF) | pixG<<8);
		        				imgsrcL[l*width+xsrc] = ((imgsrcL[l*width+xsrc] & 0xFFFFFF00) | pixG);
		        			}
		        		}
		        		// move to next x at outer x scanline loop
    					continue loop;
    				}
    				// move through array of differences we built in above scan loops, determine smallest
       				// now calculate the one closest to zero from sum of differences
    				// rank = 0
    				++far;
    				int drank = Integer.MAX_VALUE;
    				// move through array of differences we built in above scan loops
    				//for(int s = xsrc+1; s < winR; s++) {
     				//for(int s = corrWinLeft; s < winR; s++) {
    				for(int s = 0; s < width; s++) {
    					//System.out.print("Y="+y+" score "+s+"="+score[s]+" ");
     					if( s == xsrc )
     						continue;
    					if (score[s] < drank) {
    						rank = s;
    						drank = score[s];
    					}
    				}
    				//System.out.println();
    				//System.out.println("------------------------");
    				//System.out.println("For x="+rank+" subject="+xsrc+" scanline y="+y+" score="+drank);
    				//System.out.println("------------------------");
    				//calc the disparity and insert into disparity map image
    				pix = (int) (Bf/Math.abs(xsrc-rank));
    				//if( pix <=0 || pix >=maxHorizontalSep)
    				//	System.out.print(xsrc+","+y+" p="+pix+"|");
    				if( pix > 255) {
    					//System.out.println("FAR PIX TRUNCATED FROM:"+xsrc+","+y+" p="+pix);
    					++trunc;
    					pix = 255;
    					// make the tile red, indicating its been truncated
        				//for( int k = xsrc-corrWinLeft; k < xsrc+corrWinRight; k++) {
        				for(int l = 0; l < corrWinSize; l++) {
        					imgsrcL[l*width+xsrc] = ((imgsrcL[l*width+xsrc] & 0x00FFFFFF) | 255<<24);
        					imgsrcL[l*width+xsrc] = ((imgsrcL[l*width+xsrc] & 0xFF000000) | 255<<16);
        				}
        				//}
    				} else {
    				// bulk pixel scanline setup
    				//for( int k = xsrc-corrWinLeft; k < xsrc+corrWinRight; k++) {
    					//for(int l = y-corrWinLeft; l < y+corrWinRight; l++) {
    					for(int l = 0; l < corrWinSize; l++) {
    						//bimage.setRGB(k, l, ((imageL.getRGB(k,l) & 0xFFFFFF) | pix<<24));	
    						// replace weighted gaussian distro value with computed disparity val in alpha channel
    						//imgsrcL[l*width+k] = ((imgsrcL[l*width+k] & 0xFFFFFF) | pix<<24);
    						// try a simulated greyscale
    						int pixG = (int) (((float)pix *.299) + ((float)pix *.587) + ((float)pix *.114));
    						imgsrcL[l*width+xsrc] = ((imgsrcL[l*width+xsrc] & 0x00FFFFFF) | 127<<24);
    						imgsrcL[l*width+xsrc] = ((imgsrcL[l*width+xsrc] & 0xFF00FFFF) | pixG<<16);
    						imgsrcL[l*width+xsrc] = ((imgsrcL[l*width+xsrc] & 0xFFFF00FF) | pixG<<8);
    						imgsrcL[l*width+xsrc] = ((imgsrcL[l*width+xsrc] & 0xFFFFFF00) | pixG);
    					}
    				//}
    				}
    				// pixel by pixel options
    				//bimage.setRGB(xsrc, y, (byte)pix);
    				//bimage.setRGB(xsrc, y, ((imageL.getRGB(xsrc,y) & 0xFFFFFF) | pix<<24));
    			} //++xsrc  move to next x subject in left image at this scanline y
    			// set the bimage chunk with processed left image band array
    			synchronized(bimage) {
    				bimage.setRGB(0, yStart, width, corrWinSize, imgsrcL, 0 , width);
    			}
    			//System.out.println("Y scan="+y);
    		} // next y
    		if( SAMPLERATE )
    			System.out.println("********IMAGE "+yStart+","+(yStart+corrWinSize)+" close="+close+" far="+far+" trunc="+trunc+"** "+(etime-System.currentTimeMillis())+" ms.*******");
	}
	/**
	 * Process the stereo images into a 3D array of RGBD indexed by [x][y][R,G,B,D]
	 * In practice the processing threads will segment the image into several regions to be processed in parallel
	 * differentiated by yStart and yEnd. corrWinSize window always guaranteed to fit evenly.
	 * A discrete cosine transform is created for both subwindows and the sum of absolute difference is used
	 * to correlate each chunk along the epipolar plane from left to right row.
	 * @param imageL Left image from bus
	 * @param imageR Right image from bus
	 * @param imageT The transformed image we use as right image
	 * @param yStart Starting Y band to process
	 * @param width width of image band
	 * @param transform The 3D rotation matrix
	 * @param convToGrey flag to perform greyscale conversion first.
	 * @param simage 3D short array to hold result. [x][y][0]=R, [x][y][1]=G, [x][y][2]=B, [x][y][3]=D
	 */
	public void processImageChunk(BufferedImage imageL, BufferedImage imageR, BufferedImage imageT, int yStart, int width, Matrix3 transform, boolean convToGrey, short[][][] simage) {	
		long etime = System.currentTimeMillis();
		// SAD - sum of absolute differences. take the diff of RGB values of
		// win 1 and 2 and sum them, result in score array
		int[] score = new int[width];
		int[] imgsrcL = new int[width*corrWinSize]; // left image scan line and kernel window
 		int[] imgsrcR = new int[width*corrWinSize]; // right image scan line and kernel window
 		float[] coeffsL = new float[corrWinSize*corrWinSize];
 		// precompute right side DCT values since we will be sweeping over them repeatedly as we march forward in left X
 		float[][] coeffsR = new float[width][corrWinSize*corrWinSize];

		// correlation window assumed symmetric left/right/up/down with odd number of elements
		// so we use corrWinLeft as top half of window array size and corrwinRight as bottom half
 		int close = 0;
 		int far = 0;
 		int xminL = 0;
 		int xmaxL = 0;
 		int xminR = 0;
 		int xmaxR = 0;
 		
 		//System.out.println("images:"+imageL+" "+imageR+" "+Thread.currentThread().getName());
		synchronized(imageL) {
			imageL.getRGB(0, yStart, width, corrWinSize, imgsrcL, 0 , width);
		}
		synchronized(imageR) {
			imageR.getRGB(0, yStart, width, corrWinSize, imgsrcR, 0 , width);
		}
		//System.out.println("Image bands "+Thread.currentThread().getName());
		
		if( transform != null ) {
			synchronized(imageT) {
				//System.out.println(imageT.getColorModel()+" orig="+imageR.getColorModel());
				for(int x = 0; x < width; x++) {
					for(int y = 0; y < corrWinSize; y++) {
						int[] tcoords = scale(x,y+yStart,transform);
						imageT.setRGB(tcoords[0], tcoords[1], imgsrcR[y*width+x]);
					}
				}
			}
			// wait for all processing bands to complete the newly transformed image
			try {
				latchTrans.await();
			} catch (InterruptedException | BrokenBarrierException e) {
				System.out.println("Barrer break, return "+Thread.currentThread().getName());
				return;
			}
			synchronized(imageT) {
				imageT.getRGB(0, yStart, width, corrWinSize, imgsrcR, 0 , width);
			}
		}

		//
		// Precompute DCTs in right image.
		// 
		for(int x = 0; x < width; x++) {
  			xminR = x;
  			xmaxR = x+corrWinSize;
				if(  x > width-corrWinSize) {
					xmaxR = width;
					xminR = width-corrWinSize;
			}
			// loop sweeps x in right image, we increment by one instead of corrWinSize as we do in outer loop
			// because we need to find every potential match. 
			//System.out.println("X scan="+x+" winsize="+winSize+" xCorrWinL="+xCorrWinL+" xCoorWinR="+xCorrWinR+"@"+Thread.currentThread().getName());
				//
			int coeffsRi = 0;
			//
			// j increments in y for right image
			for(int j = 0; j < corrWinSize; j++) {
				// recall; symmetric left/right up down
				// search at current pixel offset left, to current pixel + offset right, applied to y
				// i increments in x, row major order
				for(int i = xminR; i < xmaxR; i++) {
					int rrgb = imgsrcR[j*width+i];
					//
					rrgb &= 0xFFFFFF;
					if( convToGrey ) {
						int rr = (rrgb & 0xFF0000) >> 24;
						int rg = (rrgb & 0x00FF00) >> 16;
    						int rb = (rrgb & 0x0000FF);
    						int pixR = rr + rg + rb;
    						//int pixR = (int) (((float)rr *.299) + ((float)rg *.587) + ((float)rb *.114));
    						coeffsR[x][coeffsRi++] = pixR;
					} else {
						coeffsR[x][coeffsRi++] = rrgb;
					}
				}
			}
			synchronized(fdct2dR) {
			fdct2dR.forward(coeffsR[x], false);
			}
		}
			//System.out.println("right image precomp "+Thread.currentThread().getName());
			// for each subject pixel X in left image, sweep the scanline for that Y, move to next subject X
			// We always process in corrWinSize chunks
			loop:
			//for(int xsrc = corrWinLeft; xsrc < width-corrWinRight; /*xsrc+=corrWinSize*/xsrc++) {
	  		for(int xsrc = 0; xsrc < width; xsrc++) {	
				// sweep the epipolar scan line for each subject pixel in left image
  	  			xminL = xsrc;
	  			xmaxL = xsrc+corrWinSize;
   				if(  xsrc > width-corrWinSize) {
   					xmaxL = width;
   					xminL = width-corrWinSize;
				}
				int rank = 0;
				int pix = 0;
			    // process left image data, fill IDCT array row major order.
				// Remember, we have extracted the corrWinSize*width band from image, and now deal with that
				int coeffsLi = 0; // DCT float array counter left
				for(int ly = 0; ly < corrWinSize; ly++) {
					for(int lx = xminL; lx < xmaxL; lx++) {
						int lrgb = imgsrcL[ly*width+lx];
						lrgb &= 0xFFFFFF;
						if( convToGrey ) {
							int lr = (lrgb & 0xFF0000) >> 24;
							int lg = (lrgb & 0x00FF00) >> 16;
    						int lb = (lrgb & 0x0000FF);
    						//System.out.println(lr+" "+lg+" "+lb);
    						int pixL = lr + lg + lb; // greyscale
    						//int pixL = (int) (((float)lr *.299) + ((float)lg *.587) + ((float)lb *.114)); // human greyscale
    						coeffsL[coeffsLi++] = pixL;
						} else {
							coeffsL[coeffsLi++] = lrgb;
						}
					}
				}
				// try DCT
				//fdct2dL.inverse(coeffsL, false);
				synchronized(fdct2dL) {
				fdct2dL.forward(coeffsL, false);
				}
				//
				// Left image window set up, now begin sweep of scanline at y in right image.
				// variable x tracks the right image x position
				// 
				int sum = 0;
    			for(int x = 0; x < width; x++) {
    				// skip the subject
    				if( x == xsrc ) {
    					score[x] = Integer.MAX_VALUE;
    					continue;
    				}
    	  			xminR = x;
    	  			xmaxR = x+corrWinSize;
       				if(  x > width-corrWinSize) {
	   					xmaxR = width;
	   					xminR = width-corrWinSize;
					}
					// Compute the sum of absolute difference SAD between the 2 matrixes representing
					// the left subject subset and right target subset
					sum = 0;
					for(int isum = 0; isum < corrWinSize*corrWinSize; isum++) {
						//sum += Math.abs(coeffsL[isum]-coeffsR[isum]);
						sum += Math.abs(coeffsL[isum]-coeffsR[x][isum]);
						// sum of squared diffs
						//sum += Math.pow((coeffsL[isum]-coeffsR[isum]),2);
					}
					// score array is sized to image width and we ignore irrelevant edge elements
					score[x] = sum;
 					// If we have a value thats as small as possible, we can assume the differences are zero and this is our target
					// This is just an optimization to save time computing smallest element
					if( sum == 0) {
						++close;
						// set sadZero to current y value to signal kernel generator to progress to next y
						//sadZero.set(y);
						rank = x;
	        			pix = (int) (Bf/Math.abs(xsrc-rank));
	        			if( pix >=maxHorizontalSep) {
	        				//System.out.println("PIX TRUNCATED FROM:"+xsrc+","+y+" p="+pix);
	        				pix = maxHorizontalSep;
	        			}
	        			synchronized(simage) {
	        				//for( int k = xsrc-corrWinLeft; k < xsrc+corrWinRight; k++) {
	        					//for(int l = 0; l < corrWinSize; l++) {
	         						simage[xsrc][yStart/*+l*/][0] = (short) ((imgsrcL[/*l*width+*/xsrc] & 0x00FF0000) >> 16 );
	        						simage[xsrc][yStart/*+l*/][1] = (short) ((imgsrcL[/*l*width+*/xsrc] & 0x0000FF00) >> 8);
	        						simage[xsrc][yStart/*+l*/][2] = (short) ((imgsrcL[/*l*width+*/xsrc] & 0x000000FF) );
	        						simage[xsrc][yStart/*+l*/][3] = (short)pix;
	        					//}
	        				//}
	        					//for(int l = y; l < y+corrWinSize; l++) {
	               				//for(int l = yStart; l < yStart+corrWinSize; l++) {
	        					//	simage[xsrc][l][0] = (short) ((imageLx2.getRGB(xsrc,l) & 0x00FF0000) >> 16 );
	        					//	simage[xsrc][l][1] = (short) ((imageLx2.getRGB(xsrc,l) & 0x0000FF00) >> 8);
	        					//	simage[xsrc][l][2] = (short) ((imageLx2.getRGB(xsrc,l) & 0x000000FF) );
	        						//if( simage[xsrc][l][3] == 0 )
	        					//		simage[xsrc][l][3] = (short)pix;
	        						//else 
	        							// average depth from last images
	        							//simage[xsrc][l][3] = (short) ((simage[xsrc][1][3]+pix)/2);
	        					//}
	        				//}
	        			}
	        			// move to next x at outer x scanline loop
						continue loop;
					}
							
				} // x
				// now calculate the one closest to zero from sum of differences
				// rank = 0
				++far;
				int drank = Integer.MAX_VALUE;
				// move through array of differences we built in above scan loops
				//for(int s = xsrc+1; s < winR; s++) {
 				//for(int s = corrWinLeft; s < winR; s++) {
				for(int s = 0; s < width; s++) {
					//System.out.print("Y="+y+" score "+s+"="+score[s]+" ");
 					if( s == xsrc )
 						continue;
					if (score[s] < drank) {
						rank = s;
						drank = score[s];
					}
				}
				//System.out.println();
				//System.out.println("------------------------");
				//System.out.println("For x="+rank+" subject="+xsrc+" scanline y="+y+" score="+drank);
				//System.out.println("------------------------");
				//calc the disparity and insert into disparity map image
				pix = (int) (Bf/Math.abs(xsrc-rank));
				if( pix >=maxHorizontalSep) {
					//System.out.println("PIX TRUNCATED FROM:"+xsrc+","+y+" p="+pix);
					pix = maxHorizontalSep;
				}
				synchronized(simage) {
						//for(int l = 0; l < corrWinSize; l++) {
							simage[xsrc][/*l+*/yStart][0] = (short) ((imgsrcL[/*l*width+*/xsrc] & 0x00FF0000) >> 16 );
							simage[xsrc][/*l+*/yStart][1] = (short) ((imgsrcL[/*l*width+*/xsrc] & 0x0000FF00) >> 8);
							simage[xsrc][/*l+*/yStart][2] = (short) ((imgsrcL[/*l*width+*/xsrc] & 0x000000FF) );
							simage[xsrc][/*l*/+yStart][3] = (short)pix;
						//}
					//}
    					//for(int l = y; l < y+corrWinSize; l++) {
   						//for(int l = yStart; l < yStart+corrWinSize; l++) {
    					//	simage[xsrc][l][0] = (short) ((imageLx2.getRGB(xsrc,l) & 0x00FF0000) >> 16 );
    					//	simage[xsrc][l][1] = (short) ((imageLx2.getRGB(xsrc,l) & 0x0000FF00) >> 8);
    					//	simage[xsrc][l][2] = (short) ((imageLx2.getRGB(xsrc,l) & 0x000000FF) );
    						//if( simage[xsrc][l][3] == 0 )
    					//		simage[xsrc][l][3] = (short)pix;
    						//else 
    							// average depth from last images
    							//simage[xsrc][l][3] = (short) ((simage[xsrc][1][3]+pix)/2);
    					//}
				}
			} //++xsrc  move to next x subject in left image at this scanline y
			//System.out.println("Y scan="+y);
		//} // next y
		if( SAMPLERATE )
			System.out.println("*****IMAGE FROM "+yStart+" to "+(yStart+corrWinSize)+" close="+close+" far="+far+" ***** "+Thread.currentThread().getName()+" *** "+(System.currentTimeMillis()-etime)+" ms.***");
	}
	
	
	/**
	 * Process the stereo images into a 3D array of RGBD indexed by [x][y][R,G,B,D] using an edge detected set of images
	 * In practice the processing threads will segment the image into several regions to be processed in parallel
	 * differentiated by yStart and yEnd.
	 * corrWinSize, which is smallest window always guaranteed to fit vertically and horizontally.
	 * A discrete cosine transform is created for both subwindows and the sum of absolute difference is used
	 * to correlate each chunk along the epipolar plane from left to right row. yStart to yEnd should be evenly divisible by corrWinSize.
	 * THIS METHOD IS TO BE USED IN A SINGLE THREAD, PROCESSING A SINGLE MAGE BAND OF corrWinSize
	 * @param imageL Left image from bus
	 * @param imageR Right image from bus
	 * @param imageRx2 
	 * @param imageLx2 
	 * @param yStart Starting Y band to process
	 * @param width width of image band
	 * @param simage 3D short array to hold result. [x][y][0]=R, [x][y][1]=G, [x][y][2]=B, [x][y][3]=D
	 */
	public void processImageChunkEdge(BufferedImage imageL, BufferedImage imageR, BufferedImage imageLx2, BufferedImage imageRx2, int yStart, int width, short[][][] simage) {
			long etime = System.currentTimeMillis();
    		// SAD - sum of absolute differences. take the diff of RGB values of
    		// win 1 and 2 and sum them, result in score array
			int[] score = new int[width];
    		int[] imgsrcL = new int[width*corrWinSize]; // left image scan line and kernel window
     		int[] imgsrcR = new int[width*corrWinSize]; // right image scan line and kernel window
     		float[] coeffsL = new float[corrWinSize*corrWinSize];
     		// precompute right side DCT values since we will be sweeping over them repeatedly as we march forward in left X
     		float[][] coeffsR = new float[width][corrWinSize*corrWinSize];
     		boolean emptyRight[] = new boolean[width]; // whether chunk was found to be devoid of edges during precompute
    		// xsrc is index for subject pixel
    		// correlation window assumed symmetric left/right/up/down with odd number of elements
    		// so we use corrWinLeft as top half of window array size and corrwinRight as bottom half
     		// this is the target area but the search area is larger if we wont hit edges of image
     		int close = 0;
     		int far = 0;
     		int xminL = 0;
     		int xmaxL = 0;
     		int xminR = 0;
     		int xmaxR = 0;
     		
 			synchronized(imageL) {
				imageL.getRGB(0, yStart, width, corrWinSize, imgsrcL, 0 , width);
			}
			synchronized(imageR) {
				imageR.getRGB(0, yStart, width, corrWinSize, imgsrcR, 0 , width);
			}

			//
			// Precompute DCTs in right image.
			// 
			for(int x = 0; x < width; x++) {
	  			xminR = x;
	  			xmaxR = x+corrWinSize;
   				if(  x > width-corrWinSize) {
   					xmaxR = width;
   					xminR = width-corrWinSize;
				}
				// loop sweeps x in right image, we increment by one instead of corrWinSize as we do in outer loop
				// because we need to find every potential match. 
				//System.out.println("X scan="+x+" winsize="+winSize+" xCorrWinL="+xCorrWinL+" xCoorWinR="+xCorrWinR+"@"+Thread.currentThread().getName());
					//
				int coeffsRi = 0;
				//
				// j increments in y for right image
				emptyRight[x] = true;
				for(int j = 0; j < corrWinSize; j++) {
					// recall; symmetric left/right up down
					// search at current pixel offset left, to current pixel + offset right, applied to y
					// i increments in x, row major order
					for(int i = xminR; i < xmaxR; i++) {
						int rrgb = imgsrcR[j*width+i];
						//
						rrgb &= 0xFFFFFF;
						//coeffsR[coeffsRi++] = rrgb;
						int rr = (rrgb & 0xFF0000) >> 24;
						int rg = (rrgb & 0x00FF00) >> 16;
						int rb = (rrgb & 0x0000FF);
						if( rb != 0 )
							emptyRight[x] = false;
						int pixR = rr + rg + rb;
						//int pixR = (int) (((float)rr *.299) + ((float)rg *.587) + ((float)rb *.114));
						coeffsR[x][coeffsRi++] = pixR;
						   // Normalize and gamma correct:
					        //double rfrr = Math.pow(rr / 255.0, 2.2);
					        //double rfgg = Math.pow(rg / 255.0, 2.2);
					        //double rfbb = Math.pow(rb / 255.0, 2.2);
					        // Calculate luminance:
					        //double rflum = 0.2126 * rfrr + 0.7152 * rfgg + 0.0722 * rfbb;
					        // Gamma compand and rescale to byte range:
					        //int pixR = (int) (255.0 * Math.pow(rflum, 1.0 / 2.2));
							// sum of absolute difference of left image weight+RGB and right image weight+RGB
							//sum += Math.abs((pixL /*| (kernel[kx][ky] << 16)*/) - (pixR /*| (kernel[kx][ky] << 16)*/));
					}
				}
				// if no edge anywhere in window, move along
				if( !emptyRight[x] ) {	
					fdct2dR.forward(coeffsR[x], false);
				}
			}
				
				
    		//for(int y = yStart; y < yEnd-corrWinSize; /*y+=corrWinSize*/y++) {
     		// y controls the starting scanline of the corrWinSize band we are retrieving from images
    	 	//for(int y = yStart; y < yEnd; y+=corrWinSize) {
				//System.out.println("y="+y+" yStart="+yStart+" yEnd="+yEnd+" width="+width+"@"+Thread.currentThread().getName());
    			//synchronized(imageL) {
    			//	imageL.getRGB(0, y, width, corrWinSize, imgsrcL, 0 , width);
    			//}
    			//synchronized(imageR) {
    			//	imageR.getRGB(0, y, width, corrWinSize, imgsrcR, 0 , width);
    			//}
 
				//
    			// for each subject pixel X in left image, sweep the scanline for that Y, move to next subject X
    			// We always process in corrWinSize chunks
    			loop:
    			//for(int xsrc = corrWinLeft; xsrc < width-corrWinRight; /*xsrc+=corrWinSize*/xsrc++) {
    	  		for(int xsrc = 0; xsrc < width; xsrc++) {	
    				// sweep the epipolar scan line for each subject pixel in left image
      	  			xminL = xsrc;
    	  			xmaxL = xsrc+corrWinSize;
       				if(  xsrc > width-corrWinSize) {
	   					xmaxL = width;
	   					xminL = width-corrWinSize;
					}/* else {
						if( xsrc < corrWinSize) {
							xminL = 0; // xminL = xsrc?, xmaxL = xsrc+corrWinSize?
							xmaxL = corrWinSize;
						}
					}*/
					int rank = 0;
					int pix = 0;
				    // process left image data, fill IDCT array row major order.
					// Remember, we have extracted the corrWinSize*width band from image, and now deal with that
					// If the chunk is all black, ignore
					boolean foundEdge = false;
					int coeffsLi = 0; // DCT float array counter left
					for(int ly = 0; ly < corrWinSize; ly++) {
						for(int lx = xminL; lx < xmaxL; lx++) {
							int lrgb = imgsrcL[ly*width+lx];
							//lrgb &= 0xFFFFFF;
							//coeffsL[coeffsLi++] = lrgb;
							int lr = (lrgb & 0xFF0000) >> 24;
							int lg = (lrgb & 0x00FF00) >> 16;
							int lb = (lrgb & 0x0000FF);
							if( lb != 0 )
								foundEdge = true;
							//System.out.println(lr+" "+lg+" "+lb);
							int pixL = lr + lg + lb; // greyscale
							//int pixL = (int) (((float)lr *.299) + ((float)lg *.587) + ((float)lb *.114)); // human greyscale
							coeffsL[coeffsLi++] = pixL;
						}
					}
					// source contains no edges
					if(!foundEdge)
						continue loop;
					// try DCT
					//fdct2dL.inverse(coeffsL, false);
					fdct2dL.forward(coeffsL, false);
					//
    				// Left image window set up, now begin sweep of scanline at y in right image.
					// variable x tracks the right image x position
					// 
 					int sum = 0;
        			for(int x = 0; x < width; x++) {
        				// skip the subject
        				//if( x == xsrc ) {
        				// skip the subject and if the right chunk starting at x is devoid of edge, skip as well
        				if( x == xsrc || emptyRight[x] ) {
        					score[x] = Integer.MAX_VALUE;
        					continue;
        				}
        	  			xminR = x;
        	  			xmaxR = x+corrWinSize;
           				if(  x > width-corrWinSize) {
    	   					xmaxR = width;
    	   					xminR = width-corrWinSize;
    					}/* else {
    						if( x < corrWinSize) {
    							xminR = 0; // xminL = xsrc?, xmaxL = xsrc+corrWinSize?
    							xmaxR = corrWinSize;
    						}
    					}*/
    					// loop sweeps x in right image, we increment by one instead of corrWinSize as we do in outer loop
    					// because we need to find every potential match. 
    					//System.out.println("X scan="+x+" winsize="+winSize+" xCorrWinL="+xCorrWinL+" xCoorWinR="+xCorrWinR+"@"+Thread.currentThread().getName());
  						//
						//int coeffsRi = 0;
						//
						// j increments in y for right image
						//foundEdge = false;
						//for(int j = 0; j < corrWinSize; j++) {
    						// recall; symmetric left/right up down
    						// search at current pixel offset left, to current pixel + offset right, applied to y
							// i increments in x, row major order
    						//for(int i = xminR; i < xmaxR; i++) {
    							//int rrgb = imgsrcR[j*width+i];
    							//
    							//rrgb &= 0xFFFFFF;
    							//coeffsR[coeffsRi++] = rrgb;
    							//int rr = (rrgb & 0xFF0000) >> 24;
    							//int rg = (rrgb & 0x00FF00) >> 16;
    							//int rb = (rrgb & 0x0000FF);
    							//if( rb != 0 )
    							//	foundEdge = true;
      							//int pixR = rr + rg + rb;
    							//int pixR = (int) (((float)rr *.299) + ((float)rg *.587) + ((float)rb *.114));
    							//coeffsR[coeffsRi++] = pixR;
    							   // Normalize and gamma correct:
    						        //double rfrr = Math.pow(rr / 255.0, 2.2);
    						        //double rfgg = Math.pow(rg / 255.0, 2.2);
    						        //double rfbb = Math.pow(rb / 255.0, 2.2);
    						        // Calculate luminance:
    						        //double rflum = 0.2126 * rfrr + 0.7152 * rfgg + 0.0722 * rfbb;
    						        // Gamma compand and rescale to byte range:
    						        //int pixR = (int) (255.0 * Math.pow(rflum, 1.0 / 2.2));
    								// sum of absolute difference of left image weight+RGB and right image weight+RGB
    								//sum += Math.abs((pixL /*| (kernel[kx][ky] << 16)*/) - (pixR /*| (kernel[kx][ky] << 16)*/));
    						//}
    					//}
						// if no edge anywhere in window, move along
						//if( !foundEdge ) {
							//score[x] = Integer.MAX_VALUE;
							//continue;
						//}
    					//fdct2R.inverse(coeffsR, false);
						//fdct2dR.forward(coeffsR, false);

    					// Compute the sum of absolute difference SAD between the 2 matrixes representing
    					// the left subject subset and right target subset
    					sum = 0;
    					for(int isum = 0; isum < corrWinSize*corrWinSize; isum++) {
    						//sum += Math.abs(coeffsL[isum]-coeffsR[isum]);
    						sum += Math.abs(coeffsL[isum]-coeffsR[x][isum]);
    						// sum of squared diffs
    						//sum += Math.pow((coeffsL[isum]-coeffsR[isum]),2);
    					}
    					/*
    						synchronized(mutex) {
    							System.out.println("*** xsrc="+xsrc+" x="+x+" y="+y);
    						for(int cil = 0; cil < kernWinSize*kernWinSize; cil++) {
    								System.out.printf(cil+"=%.5f ", coeffsL[cil]);
    						}
    						System.out.println("\r\n---");
    						for(int cir = 0; cir < kernWinSize*kernWinSize; cir++) {
    								System.out.printf(cir+"=%.5f ", coeffsR[cir]);
    						}
    						System.out.println("\r\n=====");
    						}
    					*/
    					// score array is sized to image width and we ignore irrelevant edge elements
    					score[x] = sum;
     					// If we have a value thats as small as possible, we can assume the differences are zero and this is our target
    					// This is just an optimization to save time computing smallest element
    					if( sum == 0) {
    						++close;
    						// set sadZero to current y value to signal kernel generator to progress to next y
    						//sadZero.set(y);
    						rank = x;
		        			pix = (int) (Bf/Math.abs(xsrc-rank));
		        			if( pix >=maxHorizontalSep) {
		        				//System.out.println("PIX TRUNCATED FROM:"+xsrc+","+y+" p="+pix);
		        				pix = maxHorizontalSep;
		        			}
		        			synchronized(simage) {
		        				//for( int k = xsrc-corrWinLeft; k < xsrc+corrWinRight; k++) {
		        					//for(int l = 0; l < corrWinSize; l++) {
		         						//simage[k][y+l][0] = (short) ((imgsrcL[l*width+k] & 0x00FF0000) >> 16 );
		        						//simage[k][y+l][1] = (short) ((imgsrcL[l*width+k] & 0x0000FF00) >> 8);
		        						//simage[k][y+l][2] = (short) ((imgsrcL[l*width+k] & 0x000000FF) );
		        						//simage[k][y+l][3] = (short)pix;
		        					//}
		        				//}
		        				//for(int k = xminL; k < xmaxL; k++) {
		        					//for(int l = y; l < y+corrWinSize; l++) {
		               				for(int l = yStart; l < yStart+corrWinSize; l++) {
		        						simage[xsrc][l][0] = (short) ((imageLx2.getRGB(xsrc,l) & 0x00FF0000) >> 16 );
		        						simage[xsrc][l][1] = (short) ((imageLx2.getRGB(xsrc,l) & 0x0000FF00) >> 8);
		        						simage[xsrc][l][2] = (short) ((imageLx2.getRGB(xsrc,l) & 0x000000FF) );
		        						//if( simage[xsrc][l][3] == 0 )
		        							simage[xsrc][l][3] = (short)pix;
		        						//else 
		        							// average depth from last images
		        							//simage[xsrc][l][3] = (short) ((simage[xsrc][1][3]+pix)/2);
		        					}
		        				//}
		        			}
		        			// move to next x at outer x scanline loop
    						continue loop;
    					}
    							
    				} // x
    				// now calculate the one closest to zero from sum of differences
    				// rank = 0
    				++far;
    				int drank = Integer.MAX_VALUE;
    				// move through array of differences we built in above scan loops
    				//for(int s = xsrc+1; s < winR; s++) {
     				//for(int s = corrWinLeft; s < winR; s++) {
    				for(int s = 0; s < width; s++) {
    					//System.out.print("Y="+y+" score "+s+"="+score[s]+" ");
     					if( s == xsrc )
     						continue;
    					if (score[s] < drank) {
    						rank = s;
    						drank = score[s];
    					}
    				}
    				//System.out.println();
    				//System.out.println("------------------------");
    				//System.out.println("For x="+rank+" subject="+xsrc+" scanline y="+y+" score="+drank);
    				//System.out.println("------------------------");
    				//calc the disparity and insert into disparity map image
    				pix = (int) (Bf/Math.abs(xsrc-rank));
    				if( pix >=maxHorizontalSep) {
    					//System.out.println("PIX TRUNCATED FROM:"+xsrc+","+y+" p="+pix);
    					pix = maxHorizontalSep;
    				}
    				synchronized(simage) {
    					//for( int k = xsrc-corrWinLeft; k < xsrc+corrWinRight; k++) {
    						//for(int l = 0; l < corrWinSize; l++) {
    							//simage[k][y+l][0] = (short) ((imgsrcL[l*width+k] & 0x00FF0000) >> 16 );
    							//simage[k][y+l][1] = (short) ((imgsrcL[l*width+k] & 0x0000FF00) >> 8);
    							//simage[k][y+l][2] = (short) ((imgsrcL[l*width+k] & 0x000000FF) );
    							//simage[k][y+l][3] = (short)pix;
    						//}
    					//}
        				//for(int k = xminL; k < xmaxL; k++) {
        					//for(int l = y; l < y+corrWinSize; l++) {
       						for(int l = yStart; l < yStart+corrWinSize; l++) {
        						simage[xsrc][l][0] = (short) ((imageLx2.getRGB(xsrc,l) & 0x00FF0000) >> 16 );
        						simage[xsrc][l][1] = (short) ((imageLx2.getRGB(xsrc,l) & 0x0000FF00) >> 8);
        						simage[xsrc][l][2] = (short) ((imageLx2.getRGB(xsrc,l) & 0x000000FF) );
        						//if( simage[xsrc][l][3] == 0 )
        							simage[xsrc][l][3] = (short)pix;
        						//else 
        							// average depth from last images
        							//simage[xsrc][l][3] = (short) ((simage[xsrc][1][3]+pix)/2);
        					}
        				//}
    				}
    			} //++xsrc  move to next x subject in left image at this scanline y
    			//System.out.println("Y scan="+y);
    		//} // next y
    		if( SAMPLERATE )
    			System.out.println("*****IMAGE FROM "+yStart+" to "+(yStart+corrWinSize)+" close="+close+" far="+far+" ***** "+Thread.currentThread().getName()+" *** "+(System.currentTimeMillis()-etime)+" ms.***");
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
	 * Generate hough transform and resultant image
	 * @param simage
	 * @param width
	 * @param height
	 * @return
	 */
	public void processHough(BufferedImage bimage, short[][][] simage, int width, int height) {
		HoughTransform3 h = new HoughTransform3(width, height, 128);
		for(int xsrc = corrWinLeft; xsrc < width-corrWinRight; xsrc+=corrWinSize) {
			for(int y = corrWinLeft; y < height-corrWinRight; y+=corrWinSize) {
				h.addPoint(xsrc,y, simage[xsrc][y][3]);
			}
		}
		//return h.getHoughArrayImage();
		 // get the lines out 
        Vector<? extends HoughElem> lines = h.getLines(0); 
 
        // draw the lines back onto the image 
        for (int j = 0; j < lines.size(); j++) { 
            HoughLine3 line = (HoughLine3) lines.elementAt(j); 
            line.draw(bimage, Color.RED.getRGB()); 
        } 
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
		for(int i = rangeL; i< rangeR; i+=corrWinSize)	{
			for(int j = corrWinLeft; j < simage[0].length-corrWinRight; j+=corrWinSize) {
				aveC += simage[i][j][3];
				++aveCN;
			}
		}
		aveC /= aveCN;
		// left part from 0 to rangeL
		aveL = 0;
		aveLN = 0;
		for(int i = corrWinLeft; i < rangeL; i+=corrWinSize)	{
			for(int j = corrWinLeft; j < simage[0].length-corrWinRight; j+=corrWinSize) {
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
		for(int i = rangeR; i < simage.length-corrWinRight; i+=corrWinSize)	{
			for(int j = corrWinLeft; j < simage[0].length-corrWinRight; j+=corrWinSize) {
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
	

	class PlayerFrame extends JPanel {
		JFrame frame;
		public PlayerFrame() {
			frame = new JFrame("Player");
			//frame.add(this, BorderLayout.CENTER);
			// Remove window title and borders
	        frame.setUndecorated(true);
	        // Disable Alt+F4 on Windows
	        frame.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
	        // Make frame full-screen
	        //frame.setExtendedState(Frame.MAXIMIZED_BOTH);
		    Container pane = frame.getContentPane();
		    pane.setLayout(new BorderLayout());
		    pane.add(headingSlider, BorderLayout.SOUTH);
		    pane.add(pitchSlider, BorderLayout.EAST);
		    pane.add(this,BorderLayout.CENTER);
	        headingSlider.addChangeListener(new ChangeListener() {
					@Override
					public void stateChanged(ChangeEvent arg0) {
						repaint();
						viewChanged = true;
					}
		        	
		    });
		    //pitchSlider.addChangeListener(e -> renderPanel.repaint());
		    pitchSlider.addChangeListener(new ChangeListener() {
		    		@Override
					public void stateChanged(ChangeEvent arg0) {
						repaint();
						viewChanged = true;
					}
		    });	        
			// remove setsize when setting frame extended state to MAXIMIZED_BOTH
	        // Display frame
			frame.pack();
			frame.setSize(new Dimension(outWidth, outHeight));
			frame.setVisible(true);
		}
		private java.awt.Image lastFrame;
		public void setLastFrame(java.awt.Image lf) {
			if( lf == null ) {
				System.out.println("-----------LAST FRAME NULL ARG SET-------------");
				return;
			}
			if( lastFrame == null ) {
				lastFrame = lf;
				return;
			}
			synchronized(lastFrame) {
				lastFrame = lf; 
			} 
		}
		//public void paint(Graphics g) {
		public void paintComponent(Graphics g) {
			super.paintComponent(g);
			if( lastFrame == null ) {
				System.out.println("-----------LAST FRAME NULL PAINT-------------");
				return;
			}
			synchronized(lastFrame) {
				//g.drawImage(lastFrame, 0, 0, lastFrame.getWidth(this), lastFrame.getHeight(this), this);
				// 3d
				Graphics2D g2 = (Graphics2D)g;
				// move 0.0 origin to center
				g2.translate(lastFrame.getWidth(this)/2,lastFrame.getHeight(this)/2);
				// end 3d
				g2.drawImage(lastFrame, -(lastFrame.getWidth(this)/2),-(lastFrame.getHeight(this)/2), lastFrame.getWidth(this), lastFrame.getHeight(this), this);
				lastFrame.flush();
			}
		}
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

}


