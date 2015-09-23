package com.neocoretechs.robocore;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.image.BufferedImage;
//import java.awt.image.MemoryImageSource;
import java.awt.image.WritableRaster;
import java.io.File;
import java.io.IOException;
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
 * display them to the panel or write out a series of image files.
 * The function depends on remapped command line param "__mode" either "display" or directory name
 * Mainly demonstrates how we can manipulate the 3 byte BGR buffer to publish to ROS or create
 * a bufferedimage from that payload.
 * @author jg
 *
 */
public class VideoListener extends AbstractNodeMain 
{
	private static boolean DEBUG = false;
    private BufferedImage image = null;
    private PlayerFrame displayPanel;
    JFrame frame;
	String mode = "display";
	String outDir = "/";
	int frames = 0;
	int[] ibuf = null;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("video_feed");
	}
	@Override
	public void onStart(final ConnectedNode connectedNode) {

		Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
		if( remaps.containsKey("__mode") )
			mode = remaps.get("__mode");
		if( mode.equals("display")) {
			frame = new JFrame("Player");
			displayPanel = new PlayerFrame();
			frame.getContentPane().add(displayPanel, BorderLayout.CENTER);
			displayPanel.setVisible(true);
			frame.pack();
			frame.setSize(new Dimension(672, 418));
			frame.setVisible(true);
		} else { // if mode is not display, look for output file directory
			outDir = remaps.get("__mode");
			System.out.println("Sending video files to :"+outDir);
		}
		final Subscriber<sensor_msgs.Image> imgsub =
				connectedNode.newSubscriber("ardrone/image_raw", sensor_msgs.Image._TYPE);
		
		imgsub.addMessageListener(new MessageListener<sensor_msgs.Image>() {

		@Override
		public void onNewMessage(Image img) {
			ChannelBuffer cb = img.getData();
			byte[] buffer = cb.array(); // 3 byte BGR
			image = new BufferedImage(img.getWidth(), img.getHeight(), BufferedImage.TYPE_3BYTE_BGR);
			WritableRaster raster = (WritableRaster) image.getRaster();
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
		    raster.setPixels(0, 0, img.getWidth(), img.getHeight(), ibuf);
		    
			if( mode.equals("display")) {
				displayPanel.lastFrame = image;
				//displayPanel.lastFrame = displayPanel.createImage(new MemoryImageSource(newImage.imageWidth
				//		, newImage.imageHeight, buffer, 0, newImage.imageWidth));
				displayPanel.invalidate();
				displayPanel.updateUI();
			} else {	
				File f = new File(outDir+"/roscoe"+(++frames)+".jpg");
				try {
					ImageIO.write(image, "jpg", f);
					if( DEBUG )
						System.out.println("Wrote: roscoe"+frames+".jpg");
				} catch (IOException e) {
					System.out.println("Cant write image roscoe"+frames+".jpg");
				}	
			}
		}
		});
		
	}

}
