package com.neocoretechs.robocore;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.awt.image.WritableRaster;
import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.ByteArrayInputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.util.Map;

import javax.imageio.ImageIO;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;


/**
 * Create a panel and receive published video images on the Ros bus from robocore/image_raw, then
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
	private static final boolean SAMPLERATE = true; // display pubs per second

    private BufferedImage image = null;
    private PlayerFrame displayPanel;
 
    ByteBuffer cb;
    byte[] buffer = new byte[0];
   
	String mode = "display";
	String outDir = "/";
	int frames = 0;
	int[] ibuf = null;
	// http
    int port = 8000;
    CircularBlockingDeque<java.awt.Image> queue = new CircularBlockingDeque<java.awt.Image>(30);
    CircularBlockingDeque<byte[]> bqueue = new CircularBlockingDeque<byte[]>(30);
    
	private int sequenceNumber,lastSequenceNumber;
	long time1;
	
	
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
			//SwingUtilities.invokeLater(() -> {
			//      displayPanel = new PlayerFrame();
			//    });
			SwingUtilities.invokeLater(new Runnable() {
			    public void run() {
			        displayPanel = new PlayerFrame();
			    }
			});
			ThreadPoolManager.getInstance().spin(new Runnable() {
				@Override
				public void run() {
			        while(true) {
			        	java.awt.Image dimage=null;
						try {
							dimage = queue.takeFirst();
						} catch (InterruptedException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
						displayPanel.setLastFrame((java.awt.Image)dimage);
						//displayPanel.lastFrame = displayPanel.createImage(new MemoryImageSource(newImage.imageWidth
						//		, newImage.imageHeight, buffer, 0, newImage.imageWidth));
						displayPanel.invalidate();
						displayPanel.updateUI();
			        }
				}
			}, "SYSTEM");
		} else {
			// Spin up a server to server multipart image types
			if( mode.equals("display_server") ) {
				ThreadPoolManager.getInstance().spin(new Runnable() {
					@Override
					public void run() {
				        while(true) {
				        	ServerSocket ssocket = null;
				        	try {
				        		ssocket = new ServerSocket(port);
				        		while (true) {
				        			Socket client_socket = ssocket.accept();
				        			ThreadPoolManager.getInstance().spin(new StandardImageConnection(client_socket), "SYSTEM");
				        		}
				        	} catch (Exception e) {
				        		System.err.println("Exception occurred: " + e);
				        	}
				        	finally {
				        		if(ssocket != null)
				        			try {
				        				ssocket.close();
				        			} catch (IOException e) {}
				        	}
				        }
					}
				}, "SYSTEM");		
			} else { // mode has output directory
				if( mode.equals("rtsp_server") ) {
					try {
						RtspServer.init();
					} catch (Exception e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				} else {
					// if mode is not display, or display_server, look for output file directory
					outDir = remaps.get("__mode");
					System.out.println("Sending video files to :"+outDir);
				}
			}
		}
		final Subscriber<sensor_msgs.Image> imgsub =
				connectedNode.newSubscriber("robocore/image_raw", sensor_msgs.Image._TYPE);
		
		imgsub.addMessageListener(new MessageListener<sensor_msgs.Image>() {
		@Override
		public void onNewMessage(sensor_msgs.Image img) {
			long slew = System.currentTimeMillis() - time1;
			if( SAMPLERATE && slew >= 1000) {
				time1 = System.currentTimeMillis();
				System.out.println("Samples per second:"+(sequenceNumber-lastSequenceNumber)+". Slew rate="+(slew-1000));
				lastSequenceNumber = sequenceNumber;
			}
			synchronized(buffer) {
				cb = img.getData();
				buffer = cb.array(); // 3 byte BGR
			}
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
			
			if( mode.equals("display")) {
				try {
					synchronized(buffer) {
						InputStream in = new ByteArrayInputStream(buffer);
						image = ImageIO.read(in);
						in.close();
					}
				} catch (IOException e1) {
					System.out.println("Could not convert image payload due to:"+e1.getMessage());
					return;
				}
				//displayPanel.setLastFrame((java.awt.Image)image);
				//displayPanel.lastFrame = displayPanel.createImage(new MemoryImageSource(newImage.imageWidth
				//		, newImage.imageHeight, buffer, 0, newImage.imageWidth));
				//displayPanel.invalidate();
				//displayPanel.updateUI();
				queue.addLast(image);
				System.gc();
			} else {
				if( mode.equals("display_server")) {
					bqueue.addLast(buffer);
				} else { 
					if( mode.equals("rtsp_server")){
						RtspServer.queue.addLast(buffer);
					} else {
						// write file for each frame
						// if its a jpeg payload this is inefficient but still format independent
						InputStream in = new ByteArrayInputStream(buffer);
						try {
							image = ImageIO.read(in);
						} catch (IOException e1) {
							System.out.println("Could not convert image payload due to:"+e1.getMessage());
							return;
						}
						File f = new File(outDir+"/roscoe"+(++frames)+".jpg");
						try {
							ImageIO.write(image, "jpg", f);
							if( DEBUG )
								System.out.println("Wrote: roscoe"+frames+".jpg");
						} catch (IOException e) {
							System.out.println("Cant write image roscoe"+frames+".jpg");
							return;
						}	
					}
				}
			}
			++sequenceNumber; // we want to inc seq regardless to see how many we drop	
		}
		});
		
	}
	
	
	class StandardImageConnection implements Runnable {
		private static final String BOUNDARY = "reposition";

		private Socket client;
		private DataInputStream in;
		private DataOutputStream out;

		public StandardImageConnection(Socket client_socket)
		{
			client = client_socket;
			try {
				in = new DataInputStream(client.getInputStream());
				out = new DataOutputStream(new BufferedOutputStream(client.getOutputStream()));
			} catch (IOException ie) {
				System.err.println("Unable to start new connection: " + ie);
				try {
					client.close();
				} catch (IOException ie2) {
				}
				return;
			}

		}

	    public void run()
		{
			try {
				out.writeBytes("HTTP/1.0 200 OK\r\n");
				out.writeBytes("Server: RoboCore Image server\r\n");
				out.writeBytes("Content-Type: multipart/x-mixed-replace;boundary=" + BOUNDARY + "\r\n");
				out.writeBytes("\r\n");
				out.writeBytes("--" + BOUNDARY + "\n");
				int bytesRead;
				byte[] barr = new byte[1024];
				while (true) {
						out.writeBytes("Content-type: image/jpeg\n\n");
						byte[] b = null;
        				try {
							b = bqueue.takeFirst();
						} catch (InterruptedException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
						//ByteArrayInputStream fis = new ByteArrayInputStream(b);
						//while ((bytesRead = fis.read(barr)) != -1) {
						//	out.write(barr, 0, bytesRead);
						//}
						//fis.close();
        				out.write(b, 0, b.length);
//	                    out.writeBytes("\n");
						out.writeBytes("--" + BOUNDARY + "\n");
						out.flush();
				}
			} catch (Exception ie) {
				try {
					in.close();
					out.close();
					client.close();
				} catch (IOException ie2) {
				}
			}
		}
	}
	
	class PlayerFrame extends JPanel {
		JFrame frame;
		public PlayerFrame() {
			frame = new JFrame("Player");
			//displayPanel = new PlayerFrame();
			//frame.getContentPane().add(displayPanel, BorderLayout.CENTER);
			//displayPanel.setVisible(true);
			frame.getContentPane().add(this, BorderLayout.CENTER);
			this.setVisible(true);
			frame.pack();
			frame.setSize(new Dimension(640, 480));
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

