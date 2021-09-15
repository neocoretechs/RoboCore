package com.neocoretechs.robocore.video;

import java.io.IOException;
import java.nio.ByteBuffer;

import java.util.Map;
import java.util.concurrent.BrokenBarrierException;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.CyclicBarrier;
import java.util.concurrent.TimeUnit;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.CountDownServiceServerListener;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import org.ros.node.topic.Subscriber;

import com.neocoretechs.bigsack.session.BigSackAdapter;
import com.neocoretechs.bigsack.session.TransactionalHashSet;
import com.neocoretechs.relatrix.DuplicateKeyException;
import com.neocoretechs.relatrix.Relatrix;
import org.ros.internal.node.server.ThreadPoolManager;
import com.neocoretechs.robocore.services.ControllerStatusMessage;
import com.neocoretechs.robocore.services.ControllerStatusMessageRequest;
import com.neocoretechs.robocore.services.ControllerStatusMessageResponse;

//import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;


/**
 * Create a database and receive published video images on the Ros bus from /stereo_msgs/StereoImage, then
 * store them
 * The function can use remapped command line param "__commitRate" int value, to change
 * the default of 100 images initiating a checkpoint transaction.<p/>
 * Image storage must be stopped and started via the service at uri cmd_store. The payload of the
 * request is either "begin" or "end".<p/> 
 * Demonstrates how we can manipulate the image buffer to store images in the categorical database.<p/>
 * @author Jonathan Groff (C) NeoCoreTechs 2021
 *
 */
public class VideoRecorderStereo extends AbstractNodeMain 
{
	private static boolean DEBUG = true;
	private static final boolean SAMPLERATE = true; // display pubs per second

    private Object mutex = new Object();

    ByteBuffer cbl;
    byte[] bufferl = new byte[0];
    ByteBuffer cbr;
    byte[] bufferr = new byte[0];
    
    double eulers[] = new double[]{0.0,0.0,0.0};
   
	String outDir = "/";
	int frames = 0;
    //CircularBlockingDeque<java.awt.Image> queue = new CircularBlockingDeque<java.awt.Image>(30);
    //CircularBlockingDeque<byte[]> bqueue = new CircularBlockingDeque<byte[]>(30);
	private int sequenceNumber,lastSequenceNumber;
	long time1;
	protected static boolean shouldStore = true;
	private static String STORE_SERVICE = "cmd_store";
	private static int MAXIMUM = 50000;
	int commitRate = 500;
	public static String DATABASE = "D:/etc/ROSCOE2/Images";
	CountDownLatch latch;
	TransactionalHashSet session = null;
	static {
		ThreadPoolManager.getInstance().init(new String[] {"SYSTEM"}, false);
	}
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("subs_storevideo");
	}
	@Override
	public void onStart(final ConnectedNode connectedNode) {
		Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
		if( remaps.containsKey("__database") )
			DATABASE = remaps.get("__database");
		try {
			//Relatrix.setTablespaceDirectory(DATABASE);
			BigSackAdapter.setTableSpaceDir(DATABASE);
			System.out.println(">> ATTEMPTING TO ACCESS "+DATABASE);
			session = BigSackAdapter.getBigSackTransactionalHashSet(StereoscopicImageBytes.class);
		} catch (IOException | IllegalAccessException e2) {
			//System.out.println("Relatrix database volume "+DATABASE+" does not exist!");
			throw new RuntimeException(e2);
		}
		final Subscriber<stereo_msgs.StereoImage> imgsub =
				connectedNode.newSubscriber("/stereo_msgs/StereoImage", stereo_msgs.StereoImage._TYPE);
		final Subscriber<sensor_msgs.Imu> subsimu = 
				connectedNode.newSubscriber("/sensor_msgs/Imu", sensor_msgs.Imu._TYPE);

		if( remaps.containsKey("__commitRate") )
			commitRate = Integer.parseInt(remaps.get("__commitRate"));
		/*
		try {
		final CountDownServiceServerListener<ControllerStatusMessageRequest, ControllerStatusMessageResponse> serviceServerListener =
			CountDownServiceServerListener.newDefault();
		final ServiceServer<ControllerStatusMessageRequest, ControllerStatusMessageResponse> serviceServer = connectedNode.newServiceServer(STORE_SERVICE, ControllerStatusMessage._TYPE,
			new ServiceResponseBuilder<ControllerStatusMessageRequest, ControllerStatusMessageResponse>() {
						@Override
						public void build(ControllerStatusMessageRequest request,ControllerStatusMessageResponse response) {	
							switch(request.getData()) {
								case "begin":
									shouldStore = true;
									response.setData("Ok");
									break;
								case "end":
									shouldStore = false;
									response.setData("Ok");
									break;
								default:
									shouldStore  = true;
									response.setData("Ok");
									break;		
							}
						}
			});	
			serviceServer.addListener(serviceServerListener);	      
			try {
				serviceServerListener.awaitMasterRegistrationSuccess(1, TimeUnit.SECONDS);
			} catch (InterruptedException e1) {
				System.out.println("STORAGE SERVICE REGISTRATION WAS INTERRUPTED");
				e1.printStackTrace();
			}
		} catch(Exception e2) {
			System.out.println("CONTROL VIA SERVICE NOT AVAILABLE");
		}
	*/
		latch = new CountDownLatch(1);
		
		ThreadPoolManager.getInstance().spin(new Runnable() {
			@Override
			public void run() {
				if(DEBUG)
					System.out.println("Entering storage loop");
				try {
					while(shouldStore) {
						try {
							latch.await();
						} catch (InterruptedException e) {
							shouldStore = false;
						}
						synchronized(mutex) {
							StereoscopicImageBytes sib = new StereoscopicImageBytes(bufferl, bufferr);
							//try {
								//Relatrix.transactionalStore(new Long(System.currentTimeMillis()), new Double(eulers[0]), sib);
								session.put(sib);
							//} catch (DuplicateKeyException e) {
								// if within 1 ms, rare but occurs
							//}
							if(sequenceNumber%commitRate == 0) {
								System.out.println("Committing at sequence "+sequenceNumber);
								//Relatrix.transactionCommit();
								session.Commit();
								session = BigSackAdapter.getBigSackTransactionalHashSet(StereoscopicImageBytes.class);
								if(MAXIMUM > 0 && sequenceNumber >= MAXIMUM)
									System.exit(0);
							}
						}
					}
		        } catch (IllegalAccessException | IOException e) {
		        	System.out.println("Storage failed for sequence number:"+sequenceNumber+" due to:"+e);
		        	e.printStackTrace();
		        }
			}
		}, "SYSTEM");
		
		/**
		 * Image extraction from bus, then image processing, then on to display section.
		 */
		imgsub.addMessageListener(new MessageListener<stereo_msgs.StereoImage>() {
		@Override
		public void onNewMessage(stereo_msgs.StereoImage img) {
			long slew = System.currentTimeMillis() - time1;
			if( SAMPLERATE && slew >= 1000) {
				time1 = System.currentTimeMillis();
				System.out.println("Frames per second:"+(sequenceNumber-lastSequenceNumber)+" Storing:"+shouldStore+". Slew rate="+(slew-1000));
				lastSequenceNumber = sequenceNumber;
			}		
			synchronized(mutex) {
					cbl = img.getData();
					bufferl = cbl.array(); // 3 byte BGR
					cbr = img.getData2();
					bufferr = cbr.array(); // 3 byte BGR
			}
			latch.countDown();
			latch = new CountDownLatch(1);
			//IntBuffer ib = cb.toByteBuffer().asIntBuffer();
			//if( DEBUG ) {
			//	System.out.println("New image set #"+sequenceNumber+" - "+img.getWidth()+","+img.getHeight()+" sizes:"+bufferl.length+", "+bufferr.length/*ib.limit()*/);
			//}
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
			
			++sequenceNumber; // we want to inc seq regardless to see how many we drop	
		}
		});
		
		subsimu.addMessageListener(new MessageListener<sensor_msgs.Imu>() {
			@Override
			public void onNewMessage(sensor_msgs.Imu message) {
				synchronized(mutex) {
					eulers = message.getOrientationCovariance();
					//System.out.println("Nav:Orientation X:"+orientation.getX()+" Y:"+orientation.getY()+" Z:"+orientation.getZ()+" W:"+orientation.getW());
					//if(DEBUG)
						//System.out.println("Nav:Eulers "+eulers[0]+" "+eulers[1]+" "+eulers[2]);
				}
			}
		});
		

		
	}
	

	
}

