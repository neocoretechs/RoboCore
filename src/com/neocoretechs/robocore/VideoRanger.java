package com.neocoretechs.robocore;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.image.BufferedImage;
//import java.awt.image.MemoryImageSource;
import java.awt.image.WritableRaster;
import java.io.File;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.util.Map;

import javax.imageio.ImageIO;
import javax.swing.JFrame;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import com.twilight.h264.decoder.AVFrame;
import com.twilight.h264.player.FrameUtils;
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
	byte[] ibuf = null;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("video_feed");
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
			ChannelBuffer cb = img.getData();
			byte[] buffer = cb.array(); // 3 byte BGR
			int W = img.getWidth();
			int H = img.getHeight();
			//System.out.println("Image reports W="+W+" H="+H);
			if( ibuf == null )
				ibuf = new byte[buffer.length];
			boolean isame = true;
			for(int i = 0; i < buffer.length; i++) {
				if( ibuf[i] != buffer[i] )
					isame = false;
				ibuf[i] = buffer[i];
			}
			if( isame )
				return;
		   
		    
			// search center line from offsets
			int colmin = 321;
			int colmax = 324; //(W/2)+50;
			byte pixmaxR = 0;
			byte pixmaxG = 0;
			byte pixmaxB = 0;
			int rowMax = 0;
			int colMax = 0;
			for(int row = 150; row < H-1; row++) {
				for(int col = colmin; col < colmax; col++) {
					int imgstart = row*3*W + 3*col;
					byte bimgstart = ibuf[imgstart];
					byte bimgstart1 =ibuf[imgstart+1];
					byte bimgstart2 = ibuf[imgstart+2];
					
					if( bimgstart > pixmaxB && bimgstart1 > pixmaxG && bimgstart2 > pixmaxR ) {	
								pixmaxB = bimgstart;
								pixmaxG = bimgstart1;
								pixmaxR = bimgstart2;
								rowMax = row;
								colMax = col;
						
					}
				}
 			}
			if( mode.equals("display")) {
				image = new BufferedImage(img.getWidth(), img.getHeight(), BufferedImage.TYPE_3BYTE_BGR);
				WritableRaster raster = (WritableRaster) image.getRaster();		
				int[] nibuff = new int[ibuf.length];
				for(int i = 0; i < ibuf.length; i++) nibuff[i] = (ibuf[i]&255);
				raster.setPixels(0, 0, img.getWidth(), img.getHeight(), nibuff);
				raster.setPixels(colMax, rowMax, 20, 20, xbuf);
				displayPanel.lastFrame = image;
				//displayPanel.lastFrame = displayPanel.createImage(new MemoryImageSource(newImage.imageWidth
				//		, newImage.imageHeight, buffer, 0, newImage.imageWidth));
				displayPanel.invalidate();
				displayPanel.updateUI();
			}
			System.out.println("Brightest @ "+rowMax+","+colMax+" pixel:"+pixmaxB+","+pixmaxG+","+pixmaxR);
		}
		});//messagelistener
		
	}//onstart

}
