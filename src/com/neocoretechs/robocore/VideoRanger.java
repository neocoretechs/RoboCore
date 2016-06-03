package com.neocoretechs.robocore;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.image.BufferedImage;
//import java.awt.image.MemoryImageSource;
import java.awt.image.WritableRaster;

import java.nio.ByteBuffer;

import java.util.Map;

import javax.swing.JFrame;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import com.twilight.h264.player.PlayerFrame;

import sensor_msgs.Image;


/**
 * Create a panel and receive published video images on the Ros bus from ardrone/image_raw, then
 * determine distance basedon laser spot focal plane tanget calc
 * The function depends on remapped command line param "__mode" either "display" or directory name
 * @author jg
 *
 */
public class VideoRanger extends AbstractNodeMain 
{
	private static boolean DEBUG = false;
    private BufferedImage image = null;
    private PlayerFrame displayPanel;
    JFrame frame;
	String mode = "display";
	String outDir = "/";
	int frames = 0;
	int[] ibuf = null;
	
	// Values used for calculating range from captured image data
	// these values are only for a specific camera and laser setup
	double	gain = 0.0024259348;	// Gain Constant used for converting pixel offset to angle in radians
	double	offset = -0.056514344;	// Offset Constant
	double	h_cm = 40.5;		// Distance between center of camera and laser
    double	range;		        // Calculated range 
	int	pixels_from_center;	// Brightest pixel location from center
						// not bottom of frame
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("subs_range");
	}
	@Override
	public void onStart(final ConnectedNode connectedNode) {

		Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
		if( remaps.containsKey("__mode") )
			mode = remaps.get("__mode");
		if( mode != null && mode.equals("display")) {
			frame = new JFrame("Player");
			displayPanel = new PlayerFrame();
			frame.getContentPane().add(displayPanel, BorderLayout.CENTER);
			displayPanel.setVisible(true);
			frame.pack();
			frame.setSize(new Dimension(672, 418));
			frame.setVisible(true);
		}
		
		final int[] xbuf = new int[1200];
		for(int i = 0; i < 1200; i++) xbuf[i]= 255;
		
		final Subscriber<sensor_msgs.Image> imgsub =
				connectedNode.newSubscriber("ardrone/image_raw", sensor_msgs.Image._TYPE);
		
		imgsub.addMessageListener(new MessageListener<sensor_msgs.Image>() {
		@Override
		public void onNewMessage(Image img) {
			ByteBuffer cb = img.getData();
			int[] buffer = cb.asIntBuffer().array();
			//image = new BufferedImage(img.getWidth(), img.getHeight(), BufferedImage.TYPE_3BYTE_BGR);
			//image = new BufferedImage(img.getWidth(), img.getHeight(), BufferedImage.TYPE_4BYTE_ABGR);
			image = new BufferedImage(img.getWidth(), img.getHeight(), BufferedImage.TYPE_INT_ARGB);
			WritableRaster raster = (WritableRaster) image.getRaster();
			int W = img.getWidth();
			int H = img.getHeight();
			//System.out.println("Image reports W="+W+" H="+H);
			if( ibuf == null )
				ibuf = new int[buffer.length];
			boolean isame = true;
			for(int i = 0; i < buffer.length; i++) {
				if( ibuf[i] != buffer[i] )
					isame = false;
				ibuf[i] = buffer[i];
			}
			if( isame )
				return;
		   
		    //System.out.println("Image size "+W+","+H);
		    
			// search center line from offsets
			// look for contrast diff
			int colmin = 329;
			int colmax = 331;
			int pixmaxR = 0;
			int pixmaxG = 0;
			int pixmaxB = 0;
			int pixmaxry = 0;
			int maxry = 0;
			int rowMax = 0;
			int colMax = 0;
			for(int row = 180; row < 380; row++) {
				for(int col = colmin; col < colmax; col++) {
					int imgstart = row*W + col;
					int bimgstart = ibuf[imgstart];
					int bimgstart1 = ibuf[imgstart+1];
					int bimgstart2 = ibuf[imgstart+2];
					if( bimgstart < 0 ) bimgstart =  ((bimgstart*-1)+128);
					if( bimgstart1 < 0 ) bimgstart1 = ((bimgstart1*-1)+128);
					if( bimgstart2 < 0 ) bimgstart2 = ((bimgstart2*-1)+128);
					
					maxry = toGrey(bimgstart, bimgstart1, bimgstart2);
					
					System.out.println("row: "+row+" col:"+col+" scale:"+maxry);
					
					//Gray = ( Max(Red, Green, Blue) + Min(Red, Green, Blue) ) / 2
					
					if( maxry > pixmaxry ) {
						/*
					}
						// check 4 quadrants for same basic pix value
						int colm = row*3*W + 3*(col-1);
						int colp = row*3*W + 3*(col+1);
						int rowm = (row-1)*3*W + 3*col;
						int rowp = (row+1)*3*W + 3*col;
						
						int p00 = ibuf[colm];
						int p01 = ibuf[colm+1];
						int p02 = ibuf[colm+2];
						if( p00 < 0 ) p00 = ((p00*-1)+128);
						if( p01 < 0 ) p01 = ((p01*-1)+128);
						if( p02 < 0 ) p02 = ((p02*-1)+128);
						int maxryp0 = toGrey(p00, p01, p02);
						
						int p10 = ibuf[colp];
						int p11 = ibuf[colp+1];
						int p12 = ibuf[colp+2];
						if( p10 < 0 ) p10 = ((p10*-1)+128);
						if( p11 < 0 ) p11 = ((p11*-1)+128);
						if( p12 < 0 ) p12 = ((p12*-1)+128);
						int maxryp1 = toGrey(p10, p11, p12);
						
						int p20 = ibuf[rowm];
						int p21 = ibuf[rowm+1];
						int p22 = ibuf[rowm+2];
						if( p20 < 0 ) p20 = ((p20*-1)+128);
						if( p21 < 0 ) p21 = ((p21*-1)+128);
						if( p22 < 0 ) p22 = ((p22*-1)+128);
						int maxryp2 = toGrey(p20, p21, p22);
						
						int p30 = ibuf[rowp];
						int p31 = ibuf[rowp+1];
						int p32 = ibuf[rowp+2];
						if( p30 < 0 ) p30 = ((p30*-1)+128);
						if( p31 < 0 ) p31 = ((p31*-1)+128);
						if( p32 < 0 ) p32 = ((p32*-1)+128);
						int maxryp3 = toGrey(p30, p31, p32);
						
						if( Math.abs(maxry-maxryp0) < 5 && Math.abs(maxry-maxryp1) < 5 && 
							Math.abs(maxry-maxryp2) < 5 && Math.abs(maxry-maxryp3) < 5) {
							*/
						
								pixmaxB = bimgstart;
								pixmaxG = bimgstart1;
								pixmaxR = bimgstart2;
								rowMax = row;
								colMax = col;
								pixmaxry = maxry;
						//}
					}
				}
 			}
			// Calculate distance of brightest pixel from center rather than bottom of frame
	        pixels_from_center = rowMax - 180;

	        // Calculate range in cm based on bright pixel location, and setup specific constants
	        range = h_cm / Math.tan(pixels_from_center * gain + offset);
	        // check invalid value
	        if( range < 0 )
	        	return;
			System.out.println("Range:"+range+". Brightest @ "+rowMax+","+colMax+" pixel:"+pixmaxB+","+pixmaxG+","+pixmaxR+" grey:"+pixmaxry);
			
			if( mode.equals("display")) {
				int dwidth = img.getWidth();
				int dheight = img.getHeight();
				//System.out.println("Image size "+dwidth+","+dheight);
				image = new BufferedImage(dwidth, dheight, BufferedImage.TYPE_INT_ARGB);
				raster = (WritableRaster) image.getRaster();		
				int[] nibuff = new int[ibuf.length];
				raster.setPixels(0, 0, dwidth, dheight, nibuff);
				if( rowMax < 390 )
					raster.setPixels(colMax, rowMax, 10, 10, xbuf);
				displayPanel.lastFrame = image;
				//displayPanel.lastFrame = displayPanel.createImage(new MemoryImageSource(newImage.imageWidth
				//		, newImage.imageHeight, buffer, 0, newImage.imageWidth));
				displayPanel.invalidate();
				displayPanel.updateUI();
			}
	
		}
		});//messagelistener
		
	}//onstart

	private int toGrey(int b, int g, int r) {
		// standard: 
		//return (int) (b + g + (r*2))/3;
		// desaturated: return (Math.max(Math.max(p32, p31), p30) +(Math.min(Math.min(p32, p31), p30)))/2;
		// R-Y algorithm to convert color image to grayscale. The conversion coefficients are:
		//	Red: 0.5;
		//	Green: 0.419;
		//	Blue: 0.081.
		return (int) (b *.419 + g * .081 + r * .5);
	}
}
