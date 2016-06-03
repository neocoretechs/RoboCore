package com.neocoretechs.robocore;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.image.BufferedImage;
import java.awt.image.WritableRaster;
import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.util.Map;

import javax.imageio.ImageIO;
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
 * display them to the panel or write out a series of image files.
 * The function depends on remapped command line param "__mode" either "display" or directory name
 * Mainly demonstrates how we can manipulate the 3 byte BGR buffer to publish to ROS or create
 * a bufferedimage from that payload.
 * @author jg
 *
 */
public class VideoListener extends AbstractNodeMain 
{
	private static boolean DEBUG = true;
    private BufferedImage image = null;
    private PlayerFrame displayPanel;
    JFrame frame;
	String mode = "display";
	String outDir = "/";
	int frames = 0;
	int[] ibuf = null;
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("subs_video");
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
			ByteBuffer cb = img.getData();
			byte[] buffer = cb.array(); // 3 byte BGR
			//IntBuffer ib = cb.toByteBuffer().asIntBuffer();
			if( DEBUG ) {
				System.out.println("New image "+img.getWidth()+","+img.getHeight()+" size:"+buffer.length/*ib.limit()*/);
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
				int iblue = buffer[ip++];
				int igreen = buffer[ip++];
				int ired = buffer[ip++];
				ibuf[iup++] = ired;
				ibuf[iup++] = igreen;
				ibuf[iup++] = iblue;
				ibuf[iup++] = ialph;
			}
			//System.out.println(ibuf.length+" "+raster.getWidth()+" "+raster.getHeight()+" "+raster.getMinX()+" "+raster.getMinY());
		    raster.setPixels(0, 0, img.getWidth(), img.getHeight(), ibuf);
			*/
			InputStream in = new ByteArrayInputStream(buffer);
			try {
				image = ImageIO.read(in);
			} catch (IOException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
			
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
