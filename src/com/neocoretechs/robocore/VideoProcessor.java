package com.neocoretechs.robocore;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Frame;
import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.text.DecimalFormat;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

import javax.imageio.ImageIO;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;
import javax.swing.WindowConstants;






import org.ros.concurrent.CircularBlockingDeque;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import com.neocoretechs.machinevision.CannyEdgeDetector;
import com.neocoretechs.machinevision.HoughTransform;
import com.neocoretechs.machinevision.ImgProcessor;

//import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;


/**
 * Create a disparity map for the left and right images taken from stereo cameras published to bus.
 * The methodology is a variation of standard stereo block matching wherein we create a correspondence window
 * of a normal 2D gaussian distribution as weight to weigh the high order 8 bits of a 32 bit RGB value of the pixels
 * in the left and right windows. We then use standard SAD (sum of absolute differences) on the weighted RGB values to discover
 * a correlation to which the disparity calculation of (focal length * baseline)/ (XRight - XLeft) is applied.
 * The edges are dealt with as non viable boundary areas that receive inaccurate or no disparity data at the width of
 * half the correlation window size.
 * @author jg (C) NeoCoreTechs 2018
 *
 */
public class VideoProcessor extends AbstractNodeMain 
{
	private static boolean DEBUG = false;
	private static final boolean SAMPLERATE = true; // display pubs per second

    private BufferedImage imageL = null;
    private BufferedImage imageR = null;
    private PlayerFrame displayPanel;
    
    ByteBuffer cbL, cbR;
    byte[] bufferL = new byte[0];
    byte[] bufferR = new byte[0];
   
	String mode = "";
	String outDir = "/";
	int frames = 0;
	int[] ibuf = null;
	// http
    int maxHorizontalSep = 127; // max pixel disparity
    final static float f = 4.4f; // focal length mm
    final static float B = 205.0f; // baseline mm
    final static int corrWinSize = 5;
    final static int corrWinLeft = 2;
    final static int corrWinRight = 3;
    CircularBlockingDeque<BufferedImage> queueL = new CircularBlockingDeque<BufferedImage>(10);
    CircularBlockingDeque<BufferedImage> queueR = new CircularBlockingDeque<BufferedImage>(10);
    BlockingQueue<int[][]> corrWins = new ArrayBlockingQueue<int[][]>(307200, true); // 640x480
    private Object mutex = new Object(); // to notify correspondence window generator thread of new image
    byte[] bqueue;
	private int sequenceNumber,lastSequenceNumber;
	long time1;
	
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("subs_videoproc");
	}
	
	@Override
	public void onStart(final ConnectedNode connectedNode) {

		Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
		if( remaps.containsKey("__mode") )
			mode = remaps.get("__mode");
		if( mode.equals("display")) {
			if( DEBUG )
				System.out.println("Pumping frames to AWT Panel");
			SwingUtilities.invokeLater(new Runnable() {
			    public void run() {
			        displayPanel = new PlayerFrame();
			    }
			});
		}
		// construct gaussian kernel with peak at 127
		final int[][] kernel = ImgProcessor.gaussianNormal(127,corrWinSize);
		/**
		 * Thread to precompute the correspondence windows for each pixel in a new image and store those
		 * to the array backed blocking queue for retrieval by the main processing thread.
		 * This thread works strictly on left source image, while the main thread processes the right image with
		 * the kernels generated here and stored on the queue.
		 */
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
		
		ThreadPoolManager.getInstance().spin(new Runnable() {
			int[] score = null;
			BufferedImage bimage = null;
				@Override
				public void run() {
			        while(true) {
			        	if( queueL.isEmpty() || queueR.isEmpty() ) {
							try {
								Thread.sleep(1);
							} catch (InterruptedException e) {}
			        	} else {
			        		try {
								imageL = queueL.takeFirst();
				        		imageR = queueR.takeFirst();
				        		// tell the correspondence window generator queue that a new image is ready
				        		synchronized(mutex) {
				        			mutex.notify();
				        		}
							} catch (InterruptedException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
							}
			        		if( score == null ) {
			        			score = new int[imageL.getWidth()];
			        			bimage = new BufferedImage(imageL.getWidth(null), imageL.getHeight(null), BufferedImage.TYPE_BYTE_GRAY);
			        		}
			        		// SAD - sum of absolute differences. take the diff of RGB values of
			        		// win 1 and 2 and sum them
			        		// int[][] imgsrc = new int[corrWinSize][corrWinSize]; // left image subject pixel and surrounding
			        		// xsrc is index for subject pixel
			        		// correlation window assumed symmetric left/right/up/down with odd number of elements
			        		for(int y = corrWinLeft; y < imageL.getHeight()-corrWinRight; y++) {
			        			// for each subject pixel X in left image, sweep the scanline for that Y, move to next subject X
			        			for(int xsrc = corrWinLeft; xsrc < imageL.getWidth()-corrWinRight; xsrc++) {
			        				// sweep the epipolar scan line for each subject pixel in left image
			        				// first create the subject pixel kernel and weight it
			        				int kx = 0;
			        				//for(int i = xsrc-corrWinLeft; i < xsrc+corrWinRight; i++) {
			        				//		int ky = 0;
			        				//		for(int j = y-corrWinLeft; j < y+corrWinRight; j++) {
			        				//			// weight the high 8 bits with our gaussian distro values which have 127 max so sign should be unaffected
			        				//			imgsrc[kx][ky] = (imageL.getRGB(i,j) & 0xFFFFFF) | (kernel[kx][ky] << 24);
			        				//			++ky;
			        				//		}
			        				//		++kx;
			        				//}
			        				int[][] imgsrc = null;
									try {
										imgsrc = corrWins.take();
									} catch (InterruptedException e) {}
			        				// now begin sweep of scanline at y beginning at 1 plus subject x
			        				for(int x = xsrc+1; x < imageR.getWidth()-corrWinRight; x++) {
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
			        								int rrgb = imageR.getRGB(i, j);
			        								rrgb &= 0xFFFFFF;
			        								// sum of absolute difference of left image weight+RGB and right image weight+RGB
			        								sum += Math.abs(imgsrc[kx][ky] - (rrgb | (kernel[kx][ky] << 24)));
			        								++ky;
			        							}
			        							// increment x of left source weighted correlation window
			        							++kx;
			        						}
			        						// score array is sized to image width and we ignore irrelevant edge elements
			        						score[x] = sum;
			        				}
			        				// now calculate the one closest to zero from sum of differences
			        				int rank = 0;
			        				int drank = Integer.MAX_VALUE;
			        				for(int s = xsrc+1; s < score.length-corrWinRight; s++) {
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
			        				int pix = (int) ((f*B)/Math.abs(xsrc-rank));
			        				//if( pix <=0 || pix >=maxHorizontalSep)
			        				//	System.out.print(xsrc+","+y+" p="+pix+"|");
			        				if( pix > 255) {
			        					//System.out.println("PIX TRUNCATED FROM:"+xsrc+","+y+" p="+pix);
			        					pix = 255;
			        				}
			        				bimage.setRGB(xsrc, y, (byte)pix);
			        			} //++xsrc  move to next x subject in left image at this scanline y
			        			//System.out.println("Y scan="+y);
			        		}
			        		System.out.println("**********END OF IMAGE***********");
			        		if( mode.equals("display") && bimage != null) {
			        				System.out.println("setting display "+bimage);
			        				displayPanel.setLastFrame(bimage);
			        				displayPanel.invalidate();
			        				displayPanel.updateUI();
			        		}
			        		imageL = null;
			        		imageR = null;
			        	}
			        	//} // mutex
	        			
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
	
	class PlayerFrame extends JPanel {
		JFrame frame;
		public PlayerFrame() {
			frame = new JFrame("Player");
			//---
			//displayPanel = new PlayerFrame();
			//frame.getContentPane().add(displayPanel, BorderLayout.CENTER);
			//displayPanel.setVisible(true);
			//---
			//frame.getContentPane().add(this, BorderLayout.CENTER);
			frame.add(this, BorderLayout.CENTER);
			// Remove window title and borders
	        frame.setUndecorated(true);
	        // Make frame topmost
	        //frame.setAlwaysOnTop(true);
	        // Disable Alt+F4 on Windows
	        frame.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
	        // Make frame full-screen
	        frame.setExtendedState(Frame.MAXIMIZED_BOTH);
	        // Display frame
			this.setVisible(true);
			frame.pack();
			//frame.setSize(new Dimension(640, 480));
			frame.setVisible(true);
		}
		private java.awt.Image lastFrame;
		public void setLastFrame(java.awt.Image lf) {
			if( lf == null ) return;
			if( lastFrame == null ) {
				lastFrame = lf;
				return;
			}
			synchronized(lastFrame) {
				//lastFrame.getGraphics().dispose();
				lastFrame = lf; 
			} 
		}
		//public void paint(Graphics g) {
		public void paintComponent(Graphics g) {
			super.paintComponent(g);
			if( lastFrame == null )
				return;
			synchronized(lastFrame) {
				g.drawImage(lastFrame, 0, 0, lastFrame.getWidth(this), lastFrame.getHeight(this), this);
				lastFrame.flush();
			}
		}
	}
}


