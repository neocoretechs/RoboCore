package com.neocoretechs.robocore.video;

import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.ByteBuffer;

import java.util.Map;
import java.util.Scanner;

import javax.imageio.ImageIO;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import com.neocoretechs.machinevision.CannyEdgeDetector;
import com.neocoretechs.relatrix.DuplicateKeyException;
import com.neocoretechs.relatrix.Result;
import com.neocoretechs.relatrix.client.RelatrixClient;
import com.neocoretechs.relatrix.client.RemoteStream;
import com.neocoretechs.robocore.SynchronizedFixedThreadPoolManager;
import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;

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
public class VideoRecorderStereoClient extends AbstractNodeMain 
{
	private static boolean DEBUG = true;
	private static boolean DEBUGDIFF = false;
	private static final boolean SAMPLERATE = true; // display pubs per second

    private Object mutex = new Object();

    ByteBuffer cbl;
    byte[] bufferl = new byte[0];
    ByteBuffer cbr;
    byte[] bufferr = new byte[0];
    int[] prevBuffer = new int[0];
    
    double eulers[] = new double[]{0.0,0.0,0.0};
   
	String outDir = "/";
	int frames = 0;
    //CircularBlockingDeque<java.awt.Image> queue = new CircularBlockingDeque<java.awt.Image>(30);
    CircularBlockingDeque<byte[]> lqueue = new CircularBlockingDeque<byte[]>(30);
    CircularBlockingDeque<byte[]> rqueue = new CircularBlockingDeque<byte[]>(30);
	private int sequenceNumber,lastSequenceNumber;
	long time1;
	protected static boolean shouldStore = true;
	private static String STORE_SERVICE = "cmd_store";
	private static int MAXIMUM = 0;
	int commitRate = 500;
	public static String DATABASE = "COREPLEX";
	public static int DATABASE_PORT = 9020;
	//CountDownLatch latch;
	RelatrixClient session = null;
	CannyEdgeDetector detector;
	BufferedImage imagel, imager;
	static {
		SynchronizedFixedThreadPoolManager.init(5, Integer.MAX_VALUE, new String[] {"VIDEORECORDERCLIENT"});
	}
	
	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("subs_storevideoclient");
	}
	
	@Override
	public void onStart(final ConnectedNode connectedNode) {

		Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
		if( remaps.containsKey("__database") )
			DATABASE = remaps.get("__database");
		if( remaps.containsKey("__databasePort") )
			DATABASE_PORT = Integer.parseInt(remaps.get("__databasePort"));
		try {
			System.out.println(">> ATTEMPTING TO ACCESS "+DATABASE+" PORT:"+DATABASE_PORT);
			session = new RelatrixClient(DATABASE, DATABASE, DATABASE_PORT);
		} catch (IOException e2) {
			//System.out.println("Relatrix database volume "+DATABASE+" does not exist!");
			throw new RuntimeException(e2);
		}
		final Subscriber<stereo_msgs.StereoImage> imgsub =
				connectedNode.newSubscriber("/stereo_msgs/StereoImageStore", stereo_msgs.StereoImage._TYPE);
		//final Subscriber<sensor_msgs.Imu> subsimu = 
				//connectedNode.newSubscriber("/sensor_msgs/Imu", sensor_msgs.Imu._TYPE);

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
		//latch = new CountDownLatch(1);
		
		SynchronizedFixedThreadPoolManager.spin(new Runnable() {
			@Override
			public void run() {
				if(DEBUG)
					System.out.println("Entering storage loop");
				byte[] lbyte = null;
				byte[] rbyte = null;
				while(shouldStore) {
					try {
						try {
							lbyte = lqueue.take();
							rbyte = rqueue.take();
						} catch (InterruptedException e) {
							shouldStore = false;
						}
						//synchronized(mutex) {
						//if(!imageDiff()) // creates imagel
						//	continue;
						//imager = createImage(bufferr);
						//byte[] bl = convertImage(imagel);
						//byte[] br = convertImage(imager);
						//if(DEBUG)
						//	System.out.println("JPG buffers to DB size ="+bl.length+" "+br.length);
						//StereoscopicImageBytes sib = new StereoscopicImageBytes(bufferl, bufferr);
						StereoscopicImageBytes sib = new StereoscopicImageBytes(lbyte, rbyte);
						//try {
						session.store(Long.valueOf(System.currentTimeMillis()), Integer.valueOf(sequenceNumber), sib);
						//session.put(sib);
						//} catch (DuplicateKeyException e) {
						// if within 1 ms, rare but occurs
						//}
						//if(sequenceNumber%commitRate == 0) {
						//System.out.println("Committing at sequence "+sequenceNumber);
						//session = new RelatrixClient(DATABASE, DATABASE, DATABASE_PORT);
						//session.Commit();
						//session = BigSackAdapter.getBigSackTransactionalHashSet(StereoscopicImageBytes.class);
						//if(MAXIMUM > 0 && sequenceNumber >= MAXIMUM) {
						//session.close();
						//shouldStore = false;
						//}
						//}
						//}
					} catch (IOException e) {
						System.out.println("Storage failed for sequence number:"+sequenceNumber+" due to:"+e);
						e.printStackTrace();
						shouldStore = false;
					}

					System.exit(1);
				}
			}
		}, "VIDEORECORDERCLIENT");

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
			//synchronized(mutex) {
					cbl = img.getData();
					bufferl = cbl.array(); // 3 byte BGR
					cbr = img.getData2();
					bufferr = cbr.array(); // 3 byte BGR
					lqueue.add(bufferl);
					rqueue.add(bufferr);
			//}
			//latch.countDown();
			//latch = new CountDownLatch(1);
			//IntBuffer ib = cb.toByteBuffer().asIntBuffer();
			//if( DEBUG ) {
				System.out.println("New image set #"+sequenceNumber+" - "+img.getWidth()+","+img.getHeight()+" sizes:"+bufferl.length+", "+bufferr.length/*ib.limit()*/);
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
		
		//subsimu.addMessageListener(new MessageListener<sensor_msgs.Imu>() {
			//@Override
			//public void onNewMessage(sensor_msgs.Imu message) {
				//synchronized(mutex) {
					//eulers = message.getOrientationCovariance();
					//System.out.println("Nav:Orientation X:"+orientation.getX()+" Y:"+orientation.getY()+" Z:"+orientation.getZ()+" W:"+orientation.getW());
					//if(DEBUG)
						//System.out.println("Nav:Eulers "+eulers[0]+" "+eulers[1]+" "+eulers[2]);
				//}
			//}
		//});
		
	}
	boolean imageDiff() {
		detector = new CannyEdgeDetector();
		detector.setLowThreshold(.75f);
		detector.setHighThreshold(1f);
		detector.setGaussianKernelWidth(32);
		detector.setGaussianKernelRadius(3);
		try {
			imagel = createImage(bufferl);
		} catch (IOException e1) {
			System.out.println("Could not convert LEFT image payload due to:"+e1.getMessage());
			return false;
		}
		//int[] currBuffer = readLuminance(imagel);
		detector.setSourceImage(imagel);
		int[] currBuffer = detector.semiProcess();
		//float[] coeffs = new float[currBuffer.length];
		//for(int i = 0; i < currBuffer.length; i++)
		//	coeffs[i] = (float)currBuffer[i];
		//FloatDCT_1D fdct1d = new FloatDCT_1D(coeffs.length);
		//fdct1d.forward(coeffs, false);
		if(prevBuffer.length == 0) {
			prevBuffer = currBuffer;
			return true; // store first
		}
		int numDiff = 0;

		for(int i = 0; i < currBuffer.length; i++) {
			if((prevBuffer[i] > 0 && currBuffer[i] <= 0) || (prevBuffer[i] <= 0 && currBuffer[i] > 0))
				++numDiff;
		}
		prevBuffer = currBuffer;
		if(DEBUGDIFF)
			System.out.println("Image diff="+numDiff+" of "+currBuffer.length+" threshold:"+(int)(Math.ceil((double)currBuffer.length * .04)));
		// % diff
		if(numDiff > (int)(Math.ceil((double)currBuffer.length * .04)))
			return true;
		return false;
	}
	
	BufferedImage createImage(byte[] imgBuff) throws IOException {
		BufferedImage bImg;
		InputStream in = new ByteArrayInputStream(bufferl);
		bImg = ImageIO.read(in);
		in.close();
		return bImg;
	}
	
	byte[] convertImage(BufferedImage bImage) throws IOException {
		ByteArrayOutputStream bos = new ByteArrayOutputStream();
		ImageIO.write(bImage, "jpg", bos );
		return bos.toByteArray();
	}
	
	/**
	 * Luma represents the achromatic image while chroma represents the color component. 
	 * In video systems such as PAL, SECAM, and NTSC, a nonlinear luma component (Y') is calculated directly 
	 * from gamma-compressed primary intensities as a weighted sum, which, although not a perfect 
	 * representation of the colorimetric luminance, can be calculated more quickly without 
	 * the gamma expansion and compression used in photometric/colorimetric calculations. 
	 * In the Y'UV and Y'IQ models used by PAL and NTSC, the rec601 luma (Y') component is computed as
	 * Math.round(0.299f * r + 0.587f * g + 0.114f * b);
	 * rec601 Methods encode 525-line 60 Hz and 625-line 50 Hz signals, both with an active region covering 
	 * 720 luminance samples and 360 chrominance samples per line. The color encoding system is known as YCbCr 4:2:2.
	 * @param r
	 * @param g
	 * @param b
	 * @return Y'
	 */
	public static int luminance(float r, float g, float b) {
		return Math.round(0.299f * r + 0.587f * g + 0.114f * b);
	}
	
	/**
	 * Fill the data array with grayscale adjusted image data from sourceImage
	 */
	public static int[] readLuminance(BufferedImage sourceImage) {
		int[] data = new int[sourceImage.getWidth()*sourceImage.getHeight()];
		int type = sourceImage.getType();
		if (type == BufferedImage.TYPE_CUSTOM || type == BufferedImage.TYPE_INT_RGB || type == BufferedImage.TYPE_INT_ARGB) {
			int[] pixels = (int[]) sourceImage.getData().getDataElements(0, 0, sourceImage.getWidth(), sourceImage.getHeight(), null);
			for (int i = 0; i < pixels.length; i++) {
				int p = pixels[i];
				int r = (p & 0xff0000) >> 16;
				int g = (p & 0xff00) >> 8;
				int b = p & 0xff;
				data[i] = luminance(r, g, b);
			}
		} else if (type == BufferedImage.TYPE_BYTE_GRAY) {
			byte[] pixels = (byte[]) sourceImage.getData().getDataElements(0, 0, sourceImage.getWidth(), sourceImage.getHeight(), null);
			for (int i = 0; i < pixels.length; i++) {
				data[i] = (pixels[i] & 0xff);
			}
		} else if (type == BufferedImage.TYPE_USHORT_GRAY) {
			short[] pixels = (short[]) sourceImage.getData().getDataElements(0, 0, sourceImage.getWidth(), sourceImage.getHeight(), null);
			for (int i = 0; i < pixels.length; i++) {
				data[i] = (pixels[i] & 0xffff) / 256;
			}
		} else if (type == BufferedImage.TYPE_3BYTE_BGR) {
            byte[] pixels = (byte[]) sourceImage.getData().getDataElements(0, 0, sourceImage.getWidth(), sourceImage.getHeight(), null);
            int offset = 0;
            int index = 0;
            for (int i = 0; i < pixels.length; i+=3) {
                int b = pixels[offset++] & 0xff;
                int g = pixels[offset++] & 0xff;
                int r = pixels[offset++] & 0xff;
                data[index++] = luminance(r, g, b);
            }
        } else {
			throw new IllegalArgumentException("Unsupported image type: " + type);
		}
		return data;
	}	
	/**
	 * Delete records from db. cmdl: local node, remote node, remote port
	 * @param argv
	 * @throws Exception
	 */
	public static void main(String[] argv) throws Exception {
		if(argv.length < 3) {
			System.out.println("Usage: java com.neocoretechs.robocore.video.VideoRecorderStereoClient <local node> <remote node> <remote port>");
			return;
		}
		System.out.println("Will delete all video database records. Proceed? hit return");
		Scanner console = new Scanner(System.in);
		console.nextLine();
		console.close();
		RelatrixClient session = new RelatrixClient(argv[0], argv[1], Integer.parseInt(argv[2]));
	    RemoteStream stream = (RemoteStream) session.findStream('*', '*', '*');
		stream.forEach(e -> {
			try {
				session.remove(((Result)e).get(0));
			} catch (IOException e1) {
				e1.printStackTrace();
			}
		});
		session.close();
	}
}

