package com.neocoretechs.robocore;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.image.BufferedImage;
//import java.awt.image.MemoryImageSource;
import java.awt.image.WritableRaster;

import javax.swing.JFrame;

import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import com.twilight.h264.player.PlayerFrame;

import sensor_msgs.Image;


/**
 * Create a panel and receive published video images on the Ros bus from ardrone/image_raw, then
 * display them to the panel.
 * Mainly demonstrates how we can manipulate the 3 byte BGR buffer to publish to ROS or create
 * a bufferedimage from that payload.
 * @author jg
 *
 */
public class VideoListener extends AbstractNodeMain 
{
    private BufferedImage image = null;
    private PlayerFrame displayPanel;
    
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("video_feed");
	}
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		JFrame frame = new JFrame("Player");
        displayPanel = new PlayerFrame();
		frame.getContentPane().add(displayPanel, BorderLayout.CENTER);
		displayPanel.setVisible(true);
		frame.pack();
		frame.setSize(new Dimension(672, 418));
		frame.setVisible(true);
		
		final Subscriber<sensor_msgs.Image> imgsub =
				connectedNode.newSubscriber("ardrone/image_raw", sensor_msgs.Image._TYPE);
		
		imgsub.addMessageListener(new MessageListener<sensor_msgs.Image>() {
		@Override
		public void onNewMessage(Image img) {
			ChannelBuffer cb = img.getData();
			byte[] buffer = cb.array(); // 3 byte BGR
			image = new BufferedImage(img.getWidth(), img.getHeight(), BufferedImage.TYPE_3BYTE_BGR);
			WritableRaster raster = (WritableRaster) image.getRaster();
			int[] ibuf = new int[buffer.length];
			for(int i = 0; i < buffer.length; i++) ibuf[i] = buffer[i];
		    raster.setPixels(0, 0, img.getWidth(), img.getHeight(), ibuf); 
			displayPanel.lastFrame = image;
			//displayPanel.lastFrame = displayPanel.createImage(new MemoryImageSource(newImage.imageWidth
			//		, newImage.imageHeight, buffer, 0, newImage.imageWidth));
			displayPanel.invalidate();
			displayPanel.updateUI();
		}
		});
		
	}

}
