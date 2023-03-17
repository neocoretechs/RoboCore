package com.neocoretechs.robocore.video;

import java.io.IOException;
import java.nio.ByteBuffer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;

import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageListener;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import com.neocoretechs.relatrix.client.RelatrixClient;
import com.neocoretechs.relatrix.client.RemoteStream;
import com.neocoretechs.relatrix.client.RemoteTailSetIterator;
import com.neocoretechs.rknn4j.RKNN;
import com.neocoretechs.rknn4j.rknn_input_output_num;
import com.neocoretechs.rknn4j.rknn_output;
import com.neocoretechs.rknn4j.rknn_sdk_version;
import com.neocoretechs.rknn4j.rknn_tensor_attr;
import com.neocoretechs.rknn4j.image.Instance;
import com.neocoretechs.rknn4j.image.detect_result;
import com.neocoretechs.rknn4j.image.detect_result_group;
import com.neocoretechs.rknn4j.runtime.Model;
import com.neocoretechs.robocore.SynchronizedFixedThreadPoolManager;
import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;

/**
 * Create a database and receive published video images on the Ros bus from /stereo_msgs/StereoImage,
 * use the NPU to do object recognition, then handle the data.
 * The function can use remapped command line param "__commitRate" int value, to change
 * the default of 100 images initiating a checkpoint transaction.<p/>
 * Image storage must be stopped and started via the service at uri cmd_store. The payload of the
 * request is either "begin" or "end".<p/> 
 * Demonstrates how we can manipulate the image buffer to store images in the categorical database.<p/>
 * @author Jonathan Groff (C) NeoCoreTechs 2021
 *
 */
public class VideoObjectRecog extends AbstractNodeMain 
{
	private static boolean DEBUG = true;
	private static boolean DEBUGDIFF = false;
	private static final boolean SAMPLERATE = true; // display pubs per second

    private static Object mutex = new Object();

    ByteBuffer cbl;
    byte[] bufferl = new byte[0];
    ByteBuffer cbr;
    byte[] bufferr = new byte[0];
    int[] prevBuffer = new int[0];
    static boolean imageReadyL = false;
    static boolean imageReadyR = false;
    
    static double eulers[] = new double[]{0.0,0.0,0.0};
   
	String outDir = "/";
	int frames = 0;
    CircularBlockingDeque<byte[]> bqueue = new CircularBlockingDeque<byte[]>(10);
	public long sequenceNumber = 0;
	public long sequenceNumber2 = 0;
	private long lastSequenceNumber;
	private long lastSequenceNumber2;
	long time1 = System.currentTimeMillis();
	long time2 = System.currentTimeMillis();
	protected static boolean shouldStore = true;
	private static String STORE_SERVICE = "cmd_store";
	private static int MAXIMUM = 0;
	int commitRate = 500;
	public static String DATABASE = "COREPLEX";
	public static int DATABASE_PORT = 9020;
	RelatrixClient session = null;
	//
	
	// NPU constants
	long ctx; // context for NPU
	boolean wantFloat = false;
	int[] widthHeightChannel; // parameters from loaded model
	int[] dimsImage = new int[] {640,480};
 	float scale_w;//(float)widthHeightChannel[0] / (float)dimsImage[0];
  	float scale_h;//(float)widthHeightChannel[1] / (float)dimsImage[1];
	String[] labels = null;
	String MODEL_DIR = "/etc/model/";
	String LABELS_FILE = "coco_80_labels_list.txt"; //YOLOv5
	String MODEL_FILE = "RK3588/yolov5s-640-640.rknn";
	//
	Model model = new Model();
	rknn_input_output_num ioNum;
	rknn_tensor_attr[] inputAttrs;
	ArrayList<rknn_tensor_attr> tensorAttrs = new ArrayList<rknn_tensor_attr>();
	// InceptSSD specific constants
	float[][] boxPriors;
	
	static String threadGroup = "VIDEOCLIENT";
	static {
		SynchronizedFixedThreadPoolManager.init(2, Integer.MAX_VALUE, new String[] {threadGroup});
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
		if( remaps.containsKey("__commitRate") )
			commitRate = Integer.parseInt(remaps.get("__commitRate"));
		if( remaps.containsKey("__modelDir") )
			MODEL_DIR = remaps.get("__modelDir");
		if( remaps.containsKey("__labelsFile") )
			LABELS_FILE = remaps.get("__labelsFile");
		if( remaps.containsKey("__modelFile") )
			MODEL_FILE = remaps.get("__modelFile");
		if(!MODEL_DIR.endsWith(("/")))
				MODEL_DIR += "/";
		try {
			//initialize the NPU with the proper model file
			initNPU(MODEL_DIR+MODEL_FILE);
			//System.out.println(">> ATTEMPTING TO ACCESS "+DATABASE+" PORT:"+DATABASE_PORT);
			//session = new RelatrixClient(DATABASE, DATABASE, DATABASE_PORT);
		} catch (IOException e2) {
			//System.out.println("Relatrix database volume "+DATABASE+" does not exist!");
			throw new RuntimeException(e2);
		}
		final Subscriber<sensor_msgs.Imu> subsimu = 
				connectedNode.newSubscriber("/sensor_msgs/Imu", sensor_msgs.Imu._TYPE);
		final Subscriber<stereo_msgs.StereoImage> imgsub =
				connectedNode.newSubscriber("/stereo_msgs/StereoImage", stereo_msgs.StereoImage._TYPE);
		final Publisher<stereo_msgs.StereoImage> imgpub =
		connectedNode.newPublisher("/stereo_msgs/ObjectDetect", stereo_msgs.StereoImage._TYPE);

		/**
		 * Image extraction from bus, then image processing, then on to display section.
		 */
		imgsub.addMessageListener(new MessageListener<stereo_msgs.StereoImage>() {
		@Override
		public void onNewMessage(stereo_msgs.StereoImage img) {
			long slew = System.currentTimeMillis() - time2;
			if( SAMPLERATE && slew >= 1000) {
				time2 = System.currentTimeMillis();
				System.out.println("Input Frames per second:"+(sequenceNumber2-lastSequenceNumber2)+" Storing:"+shouldStore+". Slew rate="+(slew-1000));
				lastSequenceNumber2 = sequenceNumber2;
			}	
			//try {
				synchronized(mutex) {
					ByteBuffer cbl = img.getData();
					byte[] bufferl = cbl.array(); // 3 byte BGR
					ByteBuffer cbr = img.getData2();
					byte[] bufferr = cbr.array(); // 3 byte BGR
					bqueue.add(bufferl);
					bqueue.add(bufferr);
				}
	
			/*} catch (IllegalAccessException | IOException e) {
				System.out.println("Storage failed for sequence number:"+sequenceNumber+" due to:"+e);
				e.printStackTrace();
				shouldStore = false;
			}
			*/
			++sequenceNumber2; // we want to inc seq regardless to see how many we drop	
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
	/**
	 * Main publishing loop. Essentially we are publishing the data in whatever state its in, using the
	 * mutex appropriate to establish critical sections. A sleep follows each publication to keep the bus arbitrated
	 * This CancellableLoop will be canceled automatically when the node shuts down
	 */
	connectedNode.executeCancellableLoop(new CancellableLoop() {
		private int sequenceNumber;
		@Override
		protected void setup() {
			sequenceNumber = 0;
		}

		@Override
		protected void loop() throws InterruptedException {
			std_msgs.Header imghead = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);
			imghead.setSeq(sequenceNumber);
			Time tst = connectedNode.getCurrentTime();
			imghead.setStamp(tst);
			imghead.setFrameId(tst.toString());
			//sensor_msgs.Image imagemess = imgpub.newMessage();
			//stereo_msgs.StereoImage imagemess = imgpub.newMessage();
			stereo_msgs.StereoImage imagemess = connectedNode.getTopicMessageFactory().newFromType(stereo_msgs.StereoImage._TYPE);
			/*
			std_msgs.Header rimghead = connectedNode.getTopicMessageFactory().newFromType(std_msgs.Header._TYPE);
			rimghead.setSeq(sequenceNumber);
			tst = connectedNode.getCurrentTime();
			rimghead.setStamp(tst);
			rimghead.setFrameId(tst.toString());
			sensor_msgs.Image rimagemess = rimgpub.newMessage();
 			*/
			try {
				byte[] rbuff = bqueue.take();
				byte[] lbuff = bqueue.take();
				Instance rimage = createImage(rbuff);
				detect_result_group rdrg = imageDiff(rimage);
				imageReadyR = true;
				Instance limage = createImage(lbuff);
				detect_result_group ldrg = imageDiff(limage);
				imageReadyL = true;
				if( SAMPLERATE && System.currentTimeMillis() - time1 >= 1000) {
					time1 = System.currentTimeMillis();
					System.out.println("Output frames per second:"+(sequenceNumber-lastSequenceNumber)/*+limage+" "+rimage*/);
					lastSequenceNumber = sequenceNumber;
				}
				if( DEBUG )
					System.out.println(sequenceNumber+":Added frame ");
				
				synchronized(mutex) {
					byte[] leftPayload = limage.detectionsToJPEGBytes(ldrg);
					byte[] rightPayload = rimage.detectionsToJPEGBytes(rdrg);
					if(leftPayload != null && rightPayload != null) {
						imagemess.setData(ByteBuffer.wrap(leftPayload));
						imagemess.setData2(ByteBuffer.wrap(rightPayload));
						imagemess.setEncoding("JPG");
						imagemess.setWidth(dimsImage[0]);
						imagemess.setHeight(dimsImage[1]);
						imagemess.setStep(dimsImage[0]);
						imagemess.setIsBigendian((byte)0);
						imagemess.setHeader(imghead);
						imgpub.publish(imagemess);
					}
					
					//
					imageReadyL = false;
					imageReadyR = false;
					if( DEBUG )
						System.out.println("Pub. Image:"+sequenceNumber);	
				}
				
				//
				//caminfomsg.setHeader(imghead);
				//caminfomsg.setWidth(imwidth);
				//caminfomsg.setHeight(imheight);
				//caminfomsg.setDistortionModel("plumb_bob");
				//caminfomsg.setK(K);
				//caminfomsg.setP(P);
				//caminfopub.publish(caminfomsg);
				//System.out.println("Pub cam:"+imagemess);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				if(DEBUG)
					System.out.println("Image(s) not ready "+sequenceNumber);
				Thread.sleep(1);
				++lastSequenceNumber; // if no good, up the last sequence to compensate for sequence increment
			}
			++sequenceNumber; // we want to inc seq regardless to see how many we drop	
		}
	});
	}
	
	void initNPU(String modelFile) throws IOException {
		byte[] bmodel = model.load(modelFile);
		long tim = System.currentTimeMillis();
		ctx = model.init(bmodel);
		if(DEBUG)
			System.out.println("Init time:"+(System.currentTimeMillis()-tim)+" ms.");
		rknn_sdk_version sdk = model.querySDK(ctx);
		ioNum = model.queryIONumber(ctx);
		if(DEBUG)
			System.out.printf("%s %s%n", sdk, ioNum);
		//BufferedImage bimage = Instance.readBufferedImage(args[1]);
		//Instance image = null;
		inputAttrs = new rknn_tensor_attr[ioNum.getN_input()];
		for(int i = 0; i < ioNum.getN_input(); i++) {
			inputAttrs[i] = model.queryInputAttrs(ctx, i);
			if(DEBUG) {
				System.out.println("Tensor input layer "+i+" attributes:");
				System.out.println(RKNN.dump_tensor_attr(inputAttrs[i]));
			}
		}
		widthHeightChannel = Model.getWidthHeightChannel(inputAttrs[0]);
		//int[] dimsImage = Instance.computeDimensions(bimage);
	 	scale_w = 1.0f;//(float)widthHeightChannel[0] / (float)dimsImage[0];
	  	scale_h = 1.0f;//(float)widthHeightChannel[1] / (float)dimsImage[1];
	  	//File fi = new File(args[1]);
	  	//byte[] imageInByte = Files.readAllBytes(fi.toPath());
	  	//image = new Instance(args[1],dimsImage[0],dimsImage[1],widthHeightChannel[2],imageInByte,widthHeightChannel[0],widthHeightChannel[1],args[1]);
		//if(widthHeightChannel[0] != dimsImage[0] || widthHeightChannel[1] != dimsImage[1]) {
		//	System.out.printf("Resizing image from %s to %s%n",Arrays.toString(dimsImage),Arrays.toString(widthHeightChannel));
		//	image = new Instance(args[1], bimage, args[1], widthHeightChannel[0], widthHeightChannel[1]);
			//ImageIO.write(image.getImage(), "jpg", new File("resizedImage.jpg"));
		//} else {
		//	image = new Instance(args[1], bimage, args[1]);
		//}
		for(int i = 0; i < ioNum.getN_output(); i++) {
			rknn_tensor_attr outputAttr = model.queryOutputAttrs(ctx, i);
			if(DEBUG) {
				System.out.println("Tensor output layer "+i+" attributes:");
				System.out.println(RKNN.dump_tensor_attr(outputAttr));
			}
			tensorAttrs.add(outputAttr);
		}
		//
		//System.out.println("Setting up I/O..");
		// no preallocation of output image buffers for YOLO, no force floating output
		// InceptionSSD required want_float = true, it has 2 layers of output vs 3 for YOLO
		tim = System.currentTimeMillis();
		if(ioNum.getN_output() == 2) { // InceptionSSD
			wantFloat = true;
		}
		labels = Model.loadLines(MODEL_DIR+LABELS_FILE);
		if(DEBUG)
			System.out.println("Total category labels="+labels.length);
		//System.out.println("Setup time:"+(System.currentTimeMillis()-tim)+" ms.");
	}
	
	detect_result_group imageDiff(Instance image) throws IOException {
		// Set input data
		if(DEBUG)
		System.out.println(ctx+" "+widthHeightChannel[0]+" "+widthHeightChannel[1]+" "+widthHeightChannel[2]+" "+inputAttrs[0].getType()+" "+
				inputAttrs[0].getFmt()+" "+image.getRGB888().length);
		
		model.setInputs(ctx,inputAttrs[0].getSize(),inputAttrs[0].getType(),inputAttrs[0].getFmt(),image.getRGB888());
		if(DEBUG)
			System.out.println("Inputs set");
		rknn_output[] outputs = model.setOutputs(ioNum.getN_output(), false, wantFloat); // last param is wantFloat, to force output to floating
		if(DEBUG)
			System.out.println("Outputs set");
		
		long tim = System.currentTimeMillis();
		model.run(ctx);
		if(DEBUG) {
			System.out.println("Run time:"+(System.currentTimeMillis()-tim)+" ms.");
			System.out.println("Getting outputs...");
		}
		
		tim = System.currentTimeMillis();
		model.getOutputs(ctx, ioNum.getN_output(), outputs);
		if(DEBUG) {
			System.out.println("Get outputs time:"+(System.currentTimeMillis()-tim)+" ms.");
			System.out.println("Outputs:"+Arrays.toString(outputs));
		}
		detect_result_group drg = new detect_result_group();
		
		if(ioNum.getN_output() == 2) { // InceptionSSD 2 layers output
			boxPriors = Model.loadBoxPriors(MODEL_DIR+"box_priors.txt",detect_result.NUM_RESULTS);
			// If wantFloat is false, we would need the zero point and scaling
			//ArrayList<Float> scales = new ArrayList<Float>();
			//ArrayList<Integer> zps = new ArrayList<Integer>();
			//for(int i = 0; i < ioNum.getN_output(); i++) {
			//	rknn_tensor_attr outputAttr = tensorAttrs.get(i);
			//	zps.add(outputAttr.getZp());
			//	scales.add(outputAttr.getScale());
			//}
			//detect_result.post_process(outputs[0].getBuf(), outputs[1].getBuf(), boxPriors,
			//		dimsImage[0], dimsImage[1], detect_result.NMS_THRESH_SSD, 
			//		scale_w, scale_h, zps, scales, drg, labels);
			detect_result.post_process(outputs[0].getBuf(), outputs[1].getBuf(), boxPriors,
					dimsImage[0], dimsImage[1], detect_result.NMS_THRESH_SSD, 
					scale_w, scale_h, drg, labels);
			//System.out.println("Detected Result Group:"+drg);
			//image.detectionsToJPEGBytes(drg);
		} else { //YOLOv5 3 layers output
			ArrayList<Float> scales = new ArrayList<Float>();
			ArrayList<Integer> zps = new ArrayList<Integer>();
			for(int i = 0; i < ioNum.getN_output(); i++) {
				rknn_tensor_attr outputAttr = tensorAttrs.get(i);
				zps.add(outputAttr.getZp());
				scales.add(outputAttr.getScale());
			}
			detect_result.post_process(outputs[0].getBuf(), outputs[1].getBuf(), outputs[2].getBuf(),
				widthHeightChannel[1], widthHeightChannel[0], detect_result.BOX_THRESH, detect_result.NMS_THRESH, 
				scale_w, scale_h, zps, scales, drg, labels);
			if(DEBUG)
				System.out.println("Detected Result Group:"+drg);
			//image.drawDetections(drg);
		}
		return drg;
		//m.destroy(ctx);
	}
	/**
	 * Generate Instance from raw JPEG image buffer with RGA
	 * @param imgBuff
	 * @return
	 * @throws IOException
	 */
	Instance createImage(byte[] imgBuff) throws IOException {
	  	return new Instance("img",dimsImage[0],dimsImage[1],widthHeightChannel[2],imgBuff,widthHeightChannel[0],widthHeightChannel[1],"img",true);
	}
	/**
	 * Translate the image Instance and detect_result_group into raw JPEG byte array
	 * @param bImage
	 * @param group
	 * @return
	 * @throws IOException
	 */
	byte[] convertImage(Instance bImage, detect_result_group group) throws IOException {
		return bImage.detectionsToJPEGBytes(group);
	}
	
	


}
