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
import java.util.concurrent.BrokenBarrierException;
import java.util.concurrent.CyclicBarrier;

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
 * of kernWinSize square in the left and right windows where we can, or corrWinSize where we approach a boundary. 
 * We perform a forward discrete cosine transform on the 2 windows to push the major features. 
 * We then use standard SAD (sum of absolute differences) on the DCT arrays to discover
 * a correlation to which the disparity calculation of (focal length * baseline)/ (XRight - XLeft) is applied.
 * The edges are dealt with as non viable boundary areas that receive inaccurate or no disparity data at the width of
 * half the correlation window size. In general we scan a larger region than each tile index as we oversample to the level
 * of the kernWinSize except at boundaries where we default to corrWinSize. We assign the depth values to a corrWinSize square
 * even where we oversample to kernWinSize where we can.
 * We break the image into 3 bands and assign each band to a processing thread. We have an option to write a file that
 * can be read by CloudCompare for testing.
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
    final static int maxHorizontalSep = camWidth; // max pixel disparity
    final static int corrWinSize = 5; // Size of eventual depth patch
    final static int corrWinLeft = 2; // number of left/up elements in corr window
    final static int corrWinRight = 3;// number of right/down elements in corr window
    // kernel window example 9, 4, 5 for 9x9 gaussian compare kernel
    final static int kernWinSize = 9; // gaussian distribution correspondence window size square. kernel window, corrWin is a subset, area to compare
    final static int kernWinLeft = 4; // number of left/up elements in kernel window
    final static int kernWinRight = 5;// number of right/down elements in kernel window

    CircularBlockingDeque<BufferedImage> queueL = new CircularBlockingDeque<BufferedImage>(10);
    CircularBlockingDeque<BufferedImage> queueR = new CircularBlockingDeque<BufferedImage>(10);

    byte[] bqueue;
	private int sequenceNumber,lastSequenceNumber;
	long time1;

	Object mutex = new Object();
	FloatDCT_2D fdct2dL = new FloatDCT_2D(corrWinSize, corrWinSize);
	FloatDCT_2D fdct2dR = new FloatDCT_2D(corrWinSize, corrWinSize);
	FloatDCT_2D fdct2dLK = new FloatDCT_2D(kernWinSize, kernWinSize);
	FloatDCT_2D fdct2dRK = new FloatDCT_2D(kernWinSize, kernWinSize);
    CannyEdgeDetector ced = null;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("subs_videoproc");
	}
	
	@Override
	public void onStart(final ConnectedNode connectedNode) {

		Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
		if( remaps.containsKey("__mode") )
			mode = remaps.get("__mode");
		if( mode.equals("display") || mode.equals("hough")) {
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
		 * Main processing thread for image data. Extract image queue elements from ROS bus and then
		 * notify waiting worker threads to process them.
		 * Wait at synch barrier for completion of all processing threads, then display disparity 
		 */
		ThreadPoolManager.getInstance().spin(new Runnable() {
			BufferedImage bimage = null;
			short[][][] simage = null; // [x][y]([r][g][b][d])
			CyclicBarrier latch = new CyclicBarrier(4);
			CyclicBarrier latchOut = new CyclicBarrier(4);
				@Override
				public void run() {
					//
					// Spin the threads for each chunk of image
					//
					ThreadPoolManager.getInstance().spin(new Runnable() {
						int yStart = corrWinLeft;
						int yEnd = 160+corrWinSize;
						@Override
						public void run() {
							try {
								if( mode.equals("display")) {
									while(true) {
										try {
											latch.await();
											// wait for new image, a reset, then image from queue, then wait at barrier
											processImageChunk3D(imageL, imageR, yStart, yEnd, bimage);
											latchOut.await();
										} catch (BrokenBarrierException e) { System.out.println("<<BARRIER BREAK>> "+this);}
									}
								} else {
									while(true) {
										try {
											latch.await();
											// wait for new image, a reset, then image from queue, then wait at barrier
											processImageChunk(imageL, imageR, yStart, yEnd, simage);
											latchOut.await();
										} catch (BrokenBarrierException e) {System.out.println("<<BARRIER BREAK>> "+this);}
									}
								}
							} catch (InterruptedException e) {}
						}
					}, "SYSTEM");
					ThreadPoolManager.getInstance().spin(new Runnable() {
						int yStart = 160+corrWinSize;
						int yEnd = 320+corrWinSize;
						@Override
						public void run() {
							try {
								if( mode.equals("display")) {
									while(true) {
										try {
											latch.await();
											processImageChunk3D(imageL, imageR, yStart, yEnd, bimage);
											latchOut.await();
										} catch (BrokenBarrierException e) {System.out.println("<<BARRIER BREAK>> "+this);}	
									}							
								} else {
									while(true) {
										try {
											latch.await();
											// wait for new image, a reset, then image from queue, then wait at barrier
											processImageChunk(imageL, imageR, yStart, yEnd, simage);
											latchOut.await();
										} catch (BrokenBarrierException e) {System.out.println("<<BARRIER BREAK>> "+this);}
									}
								}
							} catch (InterruptedException e) {}
						}
					}, "SYSTEM");
					ThreadPoolManager.getInstance().spin(new Runnable() {
						int yStart = 320+corrWinSize;
						int yEnd = 480;
						@Override
						public void run() {
							try {
								if( mode.equals("display")) {
									while(true) {
										try {
											latch.await();
											processImageChunk3D(imageL, imageR, yStart, yEnd, bimage);
											latchOut.await();
										} catch (BrokenBarrierException e) {System.out.println("<<BARRIER BREAK>> "+this);}
									}
								} else {
									while(true) {
										try {
											latch.await();
											// wait for new image, a reset, then image from queue, then wait at barrier
											processImageChunk(imageL, imageR, yStart, yEnd, simage);
											latchOut.await();
										} catch (BrokenBarrierException e) {System.out.println("<<BARRIER BREAK>> "+this);}
									}
								}
							} catch (InterruptedException e) {}
						}
					}, "SYSTEM");
	
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
				        		if( mode.equals("display") ) {
				        			imageL = imageLx;
				        			imageR = imageRx;
				        			//score = new int[imageL.getWidth()];
				        			if( bimage == null ) {
				        				//bimage = new BufferedImage(outWidth, outHeight, BufferedImage.TYPE_INT_ARGB);
				        				bimage = new BufferedImage(outWidth, outHeight, BufferedImage.TYPE_INT_RGB);
				        				synchronized(bimage) {
			        						Graphics g = bimage.getGraphics();
			        						g.setColor(Color.WHITE);
			        						g.fillRect(0, 0, outWidth, outHeight);
			        					}
				        			}
			        				System.out.println("setting display "+bimage);
			        				displayPanel.setLastFrame(bimage);
			        				displayPanel.invalidate();
			        				displayPanel.updateUI();
			        				if( viewChanged) {
			        					viewChanged = false;
			        					synchronized(bimage) {
			        						Graphics g = bimage.getGraphics();
			        						g.setColor(Color.WHITE);
			        						g.fillRect(0, 0, outWidth, outHeight);
			        					}
			        				}
				        		} else {
				        			if( mode.equals("hough")) { // we need both
					        			imageL = imageLx;
					        			imageR = imageRx;
				        				if( bimage == null )
					        				bimage = new BufferedImage(outWidth, outHeight, BufferedImage.TYPE_INT_ARGB);
				        				if( simage == null)
				        					simage = new short[imageL.getWidth()][imageL.getHeight()][4];
				        				latch.await();
				        				latchOut.await();
				        				processHough(bimage, simage, imageL.getWidth(), imageL.getHeight());
				        				displayPanel.setLastFrame(bimage);
				        				displayPanel.invalidate();
				        				displayPanel.updateUI();
				        			} else {
				        				imageL = imageLx;
				        				imageR = imageRx;
				        				/*
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
				        	     		*/
				        				if( simage == null)
				        					simage = new short[imageL.getWidth()][imageL.getHeight()][4];
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
				        				//navigate(simage, 213, 426);
				        			}
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
	        			
			        }
				}
		
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
	 * @param yEnd Ending Y band to process
	 * @param simage buffered image to hold 3D rendering
	 * @param kernel Gaussian weighted kernel for correlation
	 */
	public void processImageChunk3D(BufferedImage imageL, BufferedImage imageR, int yStart, int yEnd, BufferedImage bimage) {
			int width = imageL.getWidth();
    		int[] score = new int[width];
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
			
			// initialize array with extremely far away depths
			for (int q = 0; q < zBuffer.length; q++) {
				zBuffer[q] = Double.NEGATIVE_INFINITY;
			}
    		// SAD - sum of absolute differences. take the diff of RGB values of
    		// win 1 and 2 and sum them
    		//int[][] imgsrc = new int[corrWinSize][corrWinSize]; // left image subject pixel and surrounding
    		int[] imgsrcL = new int[width*corrWinSize]; // left image scan line and corr window
     		int[] imgsrcR = new int[width*corrWinSize]; // right image scan line and corr window
    		// xsrc is index for subject pixel
    		// correlation window assumed symmetric left/right/up/down with odd number of elements
    		// so we use corrWinLeft as top half of array size and corrwinRight as bottom half
     		int close = 0;
     		int far = 0;
    		for(int y = yStart; y < yEnd; y+=corrWinSize/*y++*/) {
    			synchronized(imageL) {
    				imageL.getRGB(0, y, width, corrWinSize, imgsrcL, 0 , width);
    			}
    			synchronized(imageR) {
    				imageR.getRGB(0, y, width, corrWinSize, imgsrcR, 0 , width);
    			}
    			// for each subject pixel X in left image, sweep the scanline for that Y, move to next subject X
    			loop:
    			for(int xsrc = corrWinLeft; xsrc < width-corrWinRight; xsrc+=corrWinSize/*xsrc++*/) {
    				// sweep the epipolar scan line for each subject pixel in left image
    				// first create the subject pixel kernel and weight it
    				int kx = 0;
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
					int rank = 0;
					int pix = 0;
					int winL = xsrc-corrWinLeft;
    				// now begin sweep of scanline at y beginning at 1 plus subject x
    				for(int x = xsrc+1; x < width-corrWinRight; x++) {
    						// imgsrc at kx,ky is target in weighted left correlation window
    						kx = 0;
    						int sum = 0;
    						// outer loop sweeps x in right image starting at subject left image x+1 - left half of
    						// correlation window size, to x+1 + right half of correlation window size
    						for(int i = x-corrWinLeft; i < x+corrWinRight; i++) {
    							int ky = 0;
    							// recall; symmetric left/right up down
    							// search at current pixel offset left, to current pixel + offset right, applied to y
    							for(int j = y-corrWinLeft; j < y+corrWinRight; j++) {
    								//int rrgb = imageR.getRGB(i, j);
    								int rrgb = imgsrcR[ky*width+i];
    								rrgb &= 0xFFFFFF;
    								// sum of absolute difference of left image weight+RGB and right image weight+RGB
    								//sum += Math.abs(imgsrc[kx][ky] - (rrgb | (kernel[kx][ky] << 24)));
    								sum += Math.abs(imgsrcL[ky*width+(winL+kx)] - (rrgb /*|(kernel[kx][ky] << 24)*/));
    								++ky;
    							}
    							// increment x of left source weighted correlation window
    							++kx;
    						}
    						// score array is sized to image width and we ignore irrelevant edge elements
    						score[x] = sum;
    						if( sum <= 127) {
    							++close;
    							// set sadZero to current y value to signal kernel generator to progress to next y
    							//sadZero.set(y);
    							rank = x;
		        				pix = (int)(Bf/Math.abs(xsrc-rank));
		        				//if( pix <=0 || pix >=maxHorizontalSep)
		        					//System.out.print(xsrc+","+y+" p="+pix+"|");
		        				//bimage.setRGB(xsrc, y, (byte)pix); //greyscale option, not so hot
		        				for( int k = xsrc-corrWinLeft; k < xsrc+corrWinRight; k++) {
		        					int m = y - corrWinLeft;
		        					for(int l = 0; l < corrWinSize; l++) {		
		        					//for(int l = y-corrWinLeft; l < y+corrWinRight; l++) {
		        						//bimage.setRGB(k, l, ((imageL.getRGB(k,l) & 0xFFFFFF) | pix<<24));
		        						// replace weighted gaussian distro value with computed disparity val and 3D xform
		        						// z depth negative since positive is toward viewer
		        						int ks, ms;
		        						if( k <= camWidth/2 )
		        							ks = (camWidth/2) - k;
		        						else
		        							ks = k - (camWidth/2);
		        						if( (m+l) <= camHeight/2 )
		        							ms = (camHeight/2) - (m+l);
		        						else
		        							ms = (m+l) - (camHeight/2);
		        						Point t = new Point(ks, ms, -pix, new Color(((imgsrcL[l*width+k] & 0xFFFFFF))));
		        						//Vertex v0 = depthToVertex(k, m+l, pix, camWidth, camHeight, FOV);
		        						//Point t = new Point(v0, new Color(((imgsrcL[l*width+k] & 0xFFFFFF))));
		               					Vertex v1 = transform.transform(t);
		               					Vertex norm = new Vertex(v1.x,v1.y,v1.z);       
		               					double normalLength = Math.sqrt(norm.x * norm.x + norm.y * norm.y + norm.z * norm.z);
		               					norm.x /= normalLength;
		               					norm.y /= normalLength;
		               					norm.z /= normalLength;
		               					double angleCos = Math.abs(norm.z);
		               					int zIndex = (int)v1.y * outWidth + (int)v1.x;
		               					//System.out.println(v1.x+","+v1.y+","+v1.z+","+zIndex+" anglecos="+angleCos);
		               					
		               					if( zIndex < zBuffer.length && zIndex >= 0) {
		               						synchronized(zBuffer) {
		               							if (zBuffer[zIndex] < v1.z) {
		               								boolean oob = false;
		               								synchronized(bimage) {
		               									try {	
		               										bimage.setRGB((int)v1.x, (int)v1.y, getShade(t.color, angleCos).getRGB());
		               									} catch(ArrayIndexOutOfBoundsException aioob) {
		               										oob = true;
		               									}
		               								}
		               								if( oob )
		               									System.out.println("Coord out of bounds:"+v1+" point:"+t+" orig:"+k+","+(m+l)+","+pix);
		               								zBuffer[zIndex] = v1.z;
		               							}
		               						}
		               					} else {
		               						System.out.println("Zindex out of bounds="+zIndex+" "+v1+" point:"+t);
		               					}
		               					
		        						//imgsrcL[l*width+k] = ((imgsrcL[l*width+k] & 0xFFFFFF) | pix<<24);
		               					++m;
		        					}
		        				}
		        				// move to next x at outer x scanline loop
    							continue loop;
    						}
    							
    				}
    				// now calculate the one closest to zero from sum of differences
    				// rank = 0
    				++far;
    				int drank = Integer.MAX_VALUE;
    				for(int s = xsrc+1; s < width-corrWinRight; s++) {
    					//System.out.print("Y="+y+" score "+s+"="+score[s]+" ");
    					if (score[s] < drank) {
    						rank = s;
    						drank = score[s];
    					} /*else {
    						if (score[s] == drank && score[s] > 0 && score[rank] < 0) {
    							//same distance to zero but positive 
    							rank = s;
    						}
    						
    					}*/
    				}
    				//calc the disparity and insert into disparity map image
    				pix = (int)(Bf/Math.abs(xsrc-rank));
    				//if( pix <=0 || pix >=maxHorizontalSep)
    				//	System.out.print(xsrc+","+y+" p="+pix+"|");
    				for( int k = xsrc-corrWinLeft; k < xsrc+corrWinRight; k++) {
    					int m = y - corrWinLeft;
    					//for(int l = y-corrWinLeft; l < y+corrWinRight; l++) {
    					for(int l = 0; l < corrWinSize; l++) {
    						//bimage.setRGB(k, l, ((imageL.getRGB(k,l) & 0xFFFFFF) | pix<<24));
    						// replace weighted gaussian distro value with computed disparity val and 3D xform
    						// x is the same axis, the y axis is depth, and z is original Y
    						//Point t = new Point((double)k, (double)m, (double)pix, new Color(((imgsrcL[l*width+k] & 0xFFFFFF))));
    						//Vertex v0 = depthToVertex(k, m+l, pix, camWidth, camHeight, FOV);
    						//Point t = new Point(v0, new Color(((imgsrcL[l*width+k] & 0xFFFFFF))));
    						// z depth made negtive because positive is toward viewer
    						int ks, ms;
    						if( k <= camWidth/2 )
    							ks = (camWidth/2) - k;
    						else
    							ks = k - (camWidth/2);
    						if( (m+l) <= camHeight/2 )
    							ms = (camHeight/2) - (m+l);
    						else
    							ms = (m+l) - (camHeight/2);
    						Point t = new Point(ks,ms, -pix, new Color(((imgsrcL[l*width+k] & 0xFFFFFF))));
           					Vertex v1 = transform.transform(t);
           					Vertex norm = new Vertex(v1.x,v1.y,v1.z);       
           					double normalLength = Math.sqrt(norm.x * norm.x + norm.y * norm.y + norm.z * norm.z);
           					norm.x /= normalLength;
           					norm.y /= normalLength;
           					norm.z /= normalLength;
           					double angleCos = Math.abs(norm.z);
           					int zIndex = (int)v1.y * outWidth + (int)v1.x;
           					//System.out.println(v1.x+","+v1.y+","+v1.z+","+zIndex+" anglecos="+angleCos);
           					
           					/*if( zIndex < zBuffer.length && zIndex >= 0) {
           						synchronized(zBuffer) {
           							if (zBuffer[zIndex] < v1.z) {*/
           								boolean oob = false;
           								synchronized(bimage) {
           									try {
           										bimage.setRGB((int)v1.x, (int)v1.y, getShade(t.color, angleCos).getRGB());
           										//bimage.setRGB((int)v1.x, (int)v1.y, t.color.getRGB());
           									} catch(ArrayIndexOutOfBoundsException aioob) {
           										oob = true;
           									}
           								}
           								if( oob )
           									System.out.println("Coord out of bounds:"+v1+" point:"+t+" orig:"+k+","+(m+l)+","+pix);
           								/*zBuffer[zIndex] = v1.z;
           							} 
           						}
           						
       						} else {
       							System.out.println("Zindex out of bounds="+zIndex+" "+v1+" point:"+t);
       						}
       						*/
       						
           					++m;
    						// replace weighted gaussian distro value with computed disparity val in alpha channel
    						//imgsrcL[l*width+k] = ((imgsrcL[l*width+k] & 0xFFFFFF) | pix<<24);
    					}
    				}
    				//bimage.setRGB(xsrc, y, (byte)pix);
    				//bimage.setRGB(xsrc, y, ((imageL.getRGB(xsrc,y) & 0xFFFFFF) | pix<<24));
    			} //++xsrc  move to next x subject in left image at this scanline y
    			//System.out.println("Y scan="+y);
    		} // next y
    		if( SAMPLERATE )
    			System.out.println("**********END OF 3D IMAGE "+yStart+","+yEnd+" close="+close+" far="+far+" ***********");
	}
	/**
	 * Process the stereo images into a new BufferedImage with simulated greyscale representing depth
	 * and uncorrelated patches colored red 
	 * In practice the processing threads will segment the image into several regions to be processed in parallel
	 * differentiated by yStart and yEnd.
	 * @param imageL Left image from bus
	 * @param imageR Right image from bus
	 * @param yStart Starting Y band to process
	 * @param yEnd Ending Y band to process
	 * @param bimage buffered image to hold result
	 * @param kernel Gaussian weighted kernel for correlation
	 */
	public void processImageChunk(BufferedImage imageL, BufferedImage imageR, int yStart, int yEnd, BufferedImage bimage) {
			int width = imageL.getWidth();
    		int[] score = new int[width];
    		// SAD - sum of absolute differences. take the diff of RGB values of
    		// win 1 and 2 and sum them
    		//int[][] imgsrc = new int[corrWinSize][corrWinSize]; // left image subject pixel and surrounding
    		int[] imgsrcL = new int[width*corrWinSize]; // left image scan line and corr window
     		int[] imgsrcR = new int[width*corrWinSize]; // right image scan line and corr window
    		// xsrc is index for subject pixel
    		// correlation window assumed symmetric left/right/up/down with odd number of elements
    		// so we use corrWinLeft as top half of array size and corrwinRight as bottom half
     		int close = 0;
     		int far = 0;
     		int trunc = 0;
    		for(int y = yStart; y < yEnd; y+=corrWinSize/*y++*/) {
    			synchronized(imageL) {
    				imageL.getRGB(0, y, width, corrWinSize, imgsrcL, 0 , width);
    			}
    			synchronized(imageR) {
    				imageR.getRGB(0, y, width, corrWinSize, imgsrcR, 0 , width);
    			}
    			// for each subject pixel X in left image, sweep the scanline for that Y, move to next subject X
    			loop:
    			for(int xsrc = corrWinLeft; xsrc < width-corrWinRight; xsrc+=corrWinSize/*xsrc++*/) {
    				// sweep the epipolar scan line for each subject pixel in left image
    				// first create the subject pixel kernel and weight it
    				int kx = 0;
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
    				/*
    				int[][] imgsrc = null;
					try {
						imgsrc = corrWins.take();
					} catch (InterruptedException e) {}
					*/
					int rank = 0;
					int pix = 0;
					int winL = xsrc-corrWinLeft;
    				// now begin sweep of scanline at y beginning at 1 plus subject x
    				for(int x = xsrc+1; x < width-corrWinRight; x++) {
    						// imgsrc at kx,ky is target in weighted left correlation window
    						kx = 0;
    						int sum = 0;
    						// outer loop sweeps x in right image starting at subject left image x+1 - left half of
    						// correlation window size, to x+1 + right half of correlation window size
    						for(int i = x-corrWinLeft; i < x+corrWinRight; i++) {
    							int ky = 0;
    							// recall; symmetric left/right up down
    							// search at current pixel offset left, to current pixel + offset right, applied to y
    							for(int j = y-corrWinLeft; j < y+corrWinRight; j++) {
    								//int rrgb = imageR.getRGB(i, j);
    								int rrgb = imgsrcR[ky*width+i];
    								rrgb &= 0xFFFFFF;
    								// sum of absolute difference of left image weight+RGB and right image weight+RGB
    								//sum += Math.abs(imgsrc[kx][ky] - (rrgb | (kernel[kx][ky] << 24)));
    								sum += Math.abs(imgsrcL[ky*width+(winL+kx)] - (rrgb /*| (kernel[kx][ky] << 24)*/));
    								++ky;
    							}
    							// increment x of left source weighted correlation window
    							++kx;
    						}
    						// score array is sized to image width and we ignore irrelevant edge elements
    						score[x] = sum;
    						// If we have a value thats as small as 127, we can assume the differences are zero and this is our target
    						// This is just an optimization to save time computing smallest element
    						if( sum <= 127) {
    							++close;
    							// set sadZero to current y value to signal kernel generator to progress to next y
    							//sadZero.set(y);
    							rank = x;
		        				pix = (int) (Bf/Math.abs(xsrc-rank));
		        				//if( pix <=0 || pix >=maxHorizontalSep)
		        					//System.out.print(xsrc+","+y+" p="+pix+"|");
		        				if( pix > 255) {
		        				//	System.out.println("CLOSE PIX TRUNCATED FROM:"+xsrc+","+y+" p="+pix);
		        					++trunc;
		        					pix = 255;
		        					// make the chunk red, indicating its been truncated
		              				for( int k = xsrc-corrWinLeft; k < xsrc+corrWinRight; k++) {
		            					for(int l = 0; l < corrWinSize; l++) {
		            						imgsrcL[l*width+k] = ((imgsrcL[l*width+k] & 0x00FFFFFF) | 127<<24);
		            						imgsrcL[l*width+k] = ((imgsrcL[l*width+k] & 0xFF000000) | 255<<16);
		            					}
		            				}
		        				} else {
		        				//bimage.setRGB(xsrc, y, (byte)pix); //pixel by pixel ersatz greyscale option, not so hot
		        				// bulk pixel scanline setup
		        				for( int k = xsrc-corrWinLeft; k < xsrc+corrWinRight; k++) {
		        					for(int l = 0; l < corrWinSize; l++) {
		        					//for(int l = y-corrWinLeft; l < y+corrWinRight; l++) {
		        						//bimage.setRGB(k, l, ((imageL.getRGB(k,l) & 0xFFFFFF) | pix<<24));
		        						// replace weighted gaussian distro value with computed disparity val in alpha channel
		        						//imgsrcL[l*width+k] = ((imgsrcL[l*width+k] & 0xFFFFFF) | pix<<24);
		        						// try a simulated greyscale
		        						int pixG = (int) (((float)pix *.299) + ((float)pix *.587) + ((float)pix *.114));
		        						imgsrcL[l*width+k] = ((imgsrcL[l*width+k] & 0x00FFFFFF) | 127<<24);
		        						imgsrcL[l*width+k] = ((imgsrcL[l*width+k] & 0xFF00FFFF) | pixG<<16);
		        						imgsrcL[l*width+k] = ((imgsrcL[l*width+k] & 0xFFFF00FF) | pixG<<8);
		        						imgsrcL[l*width+k] = ((imgsrcL[l*width+k] & 0xFFFFFF00) | pixG);
		        					}
		        				}
		        				}
		        				// move to next x at outer x scanline loop
    							continue loop;
    						}
    							
    				}
    				// now calculate the one closest to zero from sum of differences
    				// rank = 0
    				++far;
    				int drank = Integer.MAX_VALUE;
    				// move through array of differences we built in above scan loops, determine smallest
    				for(int s = xsrc+1; s < width-corrWinRight; s++) {
    					//System.out.print("Y="+y+" score "+s+"="+score[s]+" ");
    					if (score[s] < drank) {
    						rank = s;
    						drank = score[s];
    					} /*else {
    						if (score[s] == drank && score[s] > 0 && score[rank] < 0) {
    							//same distance to zero but positive 
    							rank = s;
    						}
    						
    					}*/
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
        				for( int k = xsrc-corrWinLeft; k < xsrc+corrWinRight; k++) {
        					for(int l = 0; l < corrWinSize; l++) {
        						imgsrcL[l*width+k] = ((imgsrcL[l*width+k] & 0x00FFFFFF) | 127<<24);
        						imgsrcL[l*width+k] = ((imgsrcL[l*width+k] & 0xFF000000) | 255<<16);
        					}
        				}
    				} else {
    				// bulk pixel scanline setup
    				for( int k = xsrc-corrWinLeft; k < xsrc+corrWinRight; k++) {
    					//for(int l = y-corrWinLeft; l < y+corrWinRight; l++) {
    					for(int l = 0; l < corrWinSize; l++) {
    						//bimage.setRGB(k, l, ((imageL.getRGB(k,l) & 0xFFFFFF) | pix<<24));	
    						// replace weighted gaussian distro value with computed disparity val in alpha channel
    						//imgsrcL[l*width+k] = ((imgsrcL[l*width+k] & 0xFFFFFF) | pix<<24);
    						// try a simulated greyscale
    						int pixG = (int) (((float)pix *.299) + ((float)pix *.587) + ((float)pix *.114));
    						imgsrcL[l*width+k] = ((imgsrcL[l*width+k] & 0x00FFFFFF) | 127<<24);
    						imgsrcL[l*width+k] = ((imgsrcL[l*width+k] & 0xFF00FFFF) | pixG<<16);
    						imgsrcL[l*width+k] = ((imgsrcL[l*width+k] & 0xFFFF00FF) | pixG<<8);
    						imgsrcL[l*width+k] = ((imgsrcL[l*width+k] & 0xFFFFFF00) | pixG);
    					}
    				}
    				}
    				// pixel by pixel options
    				//bimage.setRGB(xsrc, y, (byte)pix);
    				//bimage.setRGB(xsrc, y, ((imageL.getRGB(xsrc,y) & 0xFFFFFF) | pix<<24));
    			} //++xsrc  move to next x subject in left image at this scanline y
    			// set the bimage chunk with processed left image band array
    			synchronized(bimage) {
    				bimage.setRGB(0, y-corrWinLeft, width, corrWinSize, imgsrcL, 0 , width);
    			}
    			//System.out.println("Y scan="+y);
    		} // next y
    		if( SAMPLERATE )
    			System.out.println("**********END OF IMAGE "+yStart+","+yEnd+" close="+close+" far="+far+" trunc="+trunc+"***********");
	}
	/**
	 * Process the stereo images into a 3D array of RGBD indexed by [x][y][R,G,B,D]
	 * In practice the processing threads will segment the image into several regions to be processed in parallel
	 * differentiated by yStart and yEnd. We will try to oversample the region to kernWinSize, but if
	 * not possible due to boundary condition, use corrWinSize, which is smallest window always guaranteed to fit, usually 5.
	 * A discrete cosine transform is created for both subwindows and the sum of absolute difference is used
	 * to correlate each chunk along the epipolar plane from left to right row.
	 * @param imageL Left image from bus
	 * @param imageR Right image from bus
	 * @param yStart Starting Y band to process
	 * @param yEnd Ending Y band to process
	 * @param simage 3D short array to hold result. [x][y][0]=R, [x][y][1]=G, [x][y][2]=B, [x][y][3]=D
	 */
	public void processImageChunk(BufferedImage imageL, BufferedImage imageR, int yStart, int yEnd, short[][][] simage) {	
			int width = imageL.getWidth();
    		int[] score = new int[width];
    		// SAD - sum of absolute differences. take the diff of RGB values of
    		// win 1 and 2 and sum them		
    		int[] imgsrcL = new int[width*kernWinSize]; // left image scan line and kernel window
     		int[] imgsrcR = new int[width*kernWinSize]; // right image scan line and kernel window
     		float[] coeffsL = null;
     		float[] coeffsR = null;
    		// xsrc is index for subject pixel
    		// correlation window assumed symmetric left/right/up/down with odd number of elements
    		// so we use corrWinLeft as top half of array size and corrwinRight as bottom half
     		// this is the target area but the search area is larger if we wont hit edges of image
     		int close = 0;
     		int far = 0;
    		for(int y = yStart; y < yEnd; y+=corrWinSize/*y++*/) {
    			int winSize = kernWinSize;
    			if( (y-yStart) < kernWinLeft || (yEnd-y) < kernWinSize ) {
    				winSize = corrWinSize;
    			}
				if( imgsrcL == null || imgsrcL.length != (width*winSize) ) {
			  		imgsrcL = new int[width*winSize]; // left image scan line and kernel window
		     		imgsrcR = new int[width*winSize]; // right image scan line and kernel window
				}
				//System.out.println("y="+y+" yStart="+yStart+" yEnd="+yEnd+" width="+width+" winSize="+winSize+"@"+Thread.currentThread().getName());
    			synchronized(imageL) {
    				imageL.getRGB(0, y, width, winSize, imgsrcL, 0 , width);
    			}
    			synchronized(imageR) {
    				imageR.getRGB(0, y, width, winSize, imgsrcR, 0 , width);
    			}
    			if( coeffsL == null || coeffsL.length != (winSize*winSize) ) {
    				coeffsR = new float[winSize*winSize];
    				coeffsL = new float[winSize*winSize];
    			}
				//
    			// for each subject pixel X in left image, sweep the scanline for that Y, move to next subject X
    			// We always process in corrWinSize chunks though if we have room we use kernWinSize chunks to compare
    			loop:
    			for(int xsrc = corrWinLeft; xsrc < width-corrWinRight; xsrc+=corrWinSize/*xsrc++*/) {
    				// sweep the epipolar scan line for each subject pixel in left image
					int rank = 0;
					int pix = 0;
					int winL;
					int winR;
					if(xsrc < kernWinLeft || xsrc > (width-kernWinRight) || winSize != kernWinSize) {
						winL = xsrc-corrWinLeft;
						winR = width-corrWinRight;
					} else {
						winL = xsrc-kernWinLeft;
						winR = width-kernWinRight;
					}
				    // process left, fill IDCT array row major order
					int coeffsLi = 0; // DCT float array counter left
					for(int ly = 0; ly < winSize; ly++) {
						for(int lx = winL; lx < winL+winSize; lx++) {
							int lrgb = imgsrcL[ly*width+lx];
							//lrgb &= 0xFFFFFF;
							//coeffsL[coeffsLi++] = lrgb;
							int lr = (lrgb & 0xFF0000) >> 24;
							int lg = (lrgb & 0x00FF00) >> 16;
							int lb = (lrgb & 0x0000FF);
							int pixL = lr + lg + lb; // greyscale
							//int pixL = (int) (((float)lr *.299) + ((float)lg *.587) + ((float)lb *.114)); // human greyscale
							coeffsL[coeffsLi++] = pixL;
						}
					}
					// try DCT
					//fdct2dL.inverse(coeffsL, false);
					//System.out.println("winsize="+winSize+"winL="+winL+" winR="+winR+" coeffsLlen="+coeffsL.length+" coeffs index"+coeffsLi);
					if(winSize == corrWinSize)
						fdct2dL.forward(coeffsL, false);
					else
						fdct2dLK.forward(coeffsL, false);
    				// now begin sweep of scanline at y beginning at 1 plus subject x
    				for(int x = xsrc+1; x < winR; x++) {
    						int sum = 0;
    						// outer loop sweeps x in right image starting at subject left image x+1 - left half of
    						// correlation window size, to x+1 + right half of correlation window size
    						int xCorrWinL;
    						int xCorrWinR;
    						if( x < kernWinLeft || x > (width-kernWinRight) || winSize != kernWinSize) {
    							xCorrWinL = x-corrWinLeft;
        						xCorrWinR = x+corrWinRight;
    						} else {
    							xCorrWinL = x-kernWinLeft;
        						xCorrWinR = x+kernWinRight;
    						}
    						//System.out.println("X scan="+x+" winsize="+winSize+" xCorrWinL="+xCorrWinL+" xCoorWinR="+xCorrWinR+"@"+Thread.currentThread().getName());
  							//
							int coeffsRi = 0;
							//
							// winL represents leftmost bound of subject window, winR, rightmost, winsize total size w or h
							// j increments in y
							for(int j = 0; j < winSize; j++) {
    							// recall; symmetric left/right up down
    							// search at current pixel offset left, to current pixel + offset right, applied to y
								// i increments in x, row major order
    							for(int i = xCorrWinL; i < xCorrWinR; i++) {
    								int rrgb = imgsrcR[j*width+i];
    								//
    								rrgb &= 0xFFFFFF;
    								//coeffsR[coeffsRi++] = rrgb;
    								int rr = (rrgb & 0xFF0000) >> 24;
    								int rg = (rrgb & 0x00FF00) >> 16;
    								int rb = (rrgb & 0x0000FF);
      								int pixR = rr + rg + rb;
    								//int pixR = (int) (((float)rr *.299) + ((float)rg *.587) + ((float)rb *.114));
    								coeffsR[coeffsRi++] = pixR;
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
    						//fdct2R.inverse(coeffsR, false);
							if(winSize == corrWinSize)
								fdct2dR.forward(coeffsR, false);
							else
								fdct2dRK.forward(coeffsR, false);
    						// Compute the sum of absolute difference SAD between the 2 matrixes representing
    						// the left subject subset and right target subset
    						sum = 0;
    						for(int isum = 0; isum < winSize*winSize; isum++) {
    							sum += Math.abs(coeffsL[isum]-coeffsR[isum]);
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
     						// If we have a value thats as small as 127, we can assume the differences are zero and this is our target
    						// This is just an optimization to save time computing smallest element
    						if( sum == 0) {
    							++close;
    							// set sadZero to current y value to signal kernel generator to progress to next y
    							//sadZero.set(y);
    							rank = x;
		        				pix = (int) (Bf/Math.abs(xsrc-rank));
		        				//if( pix <=0 || pix >=maxHorizontalSep)
		        					//System.out.print(xsrc+","+y+" p="+pix+"|");
		        				//if( pix > 255) {
		        					//System.out.println("PIX TRUNCATED FROM:"+xsrc+","+y+" p="+pix);
		        					//pix = 255;
		        				//}
		        				synchronized(simage) {
		        				for( int k = xsrc-corrWinLeft; k < xsrc+corrWinRight; k++) {
		        					for(int l = 0; l < corrWinSize; l++) {
		         						simage[k][y+l][0] = (short) ((imgsrcL[l*width+k] & 0x00FF0000) >> 16 );
		        						simage[k][y+l][1] = (short) ((imgsrcL[l*width+k] & 0x0000FF00) >> 8);
		        						simage[k][y+l][2] = (short) ((imgsrcL[l*width+k] & 0x000000FF) );
		         						//simage[k][y+l][0] = (short) 255;
		        						//simage[k][y+l][1] = (short) 0;
		        						//simage[k][y+l][2] = (short) 0;
		        						simage[k][y+l][3] = (short)pix;
		        					}
		        				}
		        				}
		        				// move to next x at outer x scanline loop
    							continue loop;
    						}
    							
    				}
    				// now calculate the one closest to zero from sum of differences
    				// rank = 0
    				++far;
    				int drank = Integer.MAX_VALUE;
    				// move through array of differences we built in above scan loops
    				for(int s = xsrc+1; s < winR; s++) {
    					//System.out.print("Y="+y+" score "+s+"="+score[s]+" ");
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
    				//if( pix > 255) {
    					//System.out.println("PIX TRUNCATED FROM:"+xsrc+","+y+" p="+pix);
    					//pix = 255;
    				//}
    				synchronized(simage) {
    				for( int k = xsrc-corrWinLeft; k < xsrc+corrWinRight; k++) {
    					for(int l = 0; l < corrWinSize; l++) {
     						simage[k][y+l][0] = (short) ((imgsrcL[l*width+k] & 0x00FF0000) >> 16 );
        					simage[k][y+l][1] = (short) ((imgsrcL[l*width+k] & 0x0000FF00) >> 8);
        					simage[k][y+l][2] = (short) ((imgsrcL[l*width+k] & 0x000000FF) );
    						simage[k][y+l][3] = (short)pix;
    					}
    				}
    				}
    			} //++xsrc  move to next x subject in left image at this scanline y
    			//System.out.println("Y scan="+y);
    		} // next y
    		if( SAMPLERATE )
    			System.out.println("**********END OF ARRAY "+yStart+","+yEnd+" close="+close+" far="+far+" ***********");
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


