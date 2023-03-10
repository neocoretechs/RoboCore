package com.neocoretechs.robocore.video;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.text.DateFormat;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import java.util.concurrent.CountDownLatch;

import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.SwingUtilities;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
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

    private static PlayerFrame displayPanel1;
    private static PlayerFrame displayPanel2;
    private static JFrame frame = null;
    private static Object mutex = new Object();

    ByteBuffer cbl;
    byte[] bufferl = new byte[0];
    ByteBuffer cbr;
    byte[] bufferr = new byte[0];
    int[] prevBuffer = new int[0];
    
    static double eulers[] = new double[]{0.0,0.0,0.0};
   
	String outDir = "/";
	int frames = 0;
    //CircularBlockingDeque<java.awt.Image> queue = new CircularBlockingDeque<java.awt.Image>(30);
    //CircularBlockingDeque<byte[]> bqueue = new CircularBlockingDeque<byte[]>(30);
	private int sequenceNumber,lastSequenceNumber;
	long time1;
	protected static boolean shouldStore = true;
	private static String STORE_SERVICE = "cmd_store";
	private static int MAXIMUM = 0;
	int commitRate = 500;
	public static String DATABASE = "COREPLEX";
	public static int DATABASE_PORT = 9020;
	static CountDownLatch latch;
	static CountDownLatch displayLatch;
	RelatrixClient session = null;
	//
	static Instance imagel;
	static Instance imager;
	static detect_result_group drgr;
	static detect_result_group drgl;
	
	// NPU constants
	long ctx; // context for NPU
	int[] widthHeightChannel; // parameters from loaded model
	int[] dimsImage = new int[] {640,480};
 	float scale_w;//(float)widthHeightChannel[0] / (float)dimsImage[0];
  	float scale_h;//(float)widthHeightChannel[1] / (float)dimsImage[1];
	boolean wantFloat = false;
	String[] labels = null;
	String MODEL_DIR = "/etc/model/";
	String LABELS_FILE = MODEL_DIR+"coco_80_labels_list.txt"; //YOLOv5
	String MODEL_FILE = MODEL_DIR+"/RK3588/yolov5s-640-640.rknn";
	//
	Model model = new Model();
	rknn_input_output_num ioNum;
	rknn_tensor_attr[] inputAttrs;
	ArrayList<rknn_tensor_attr> tensorAttrs = new ArrayList<rknn_tensor_attr>();
	// InceptSSD specific constants
	float[][] boxPriors;
	
	static {
		SynchronizedFixedThreadPoolManager.init(1, Integer.MAX_VALUE, new String[] {"VIDEORECORDERCLIENT"});
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
		final Subscriber<stereo_msgs.StereoImage> imgsub =
				connectedNode.newSubscriber("/stereo_msgs/StereoImage", stereo_msgs.StereoImage._TYPE);
		final Subscriber<sensor_msgs.Imu> subsimu = 
				connectedNode.newSubscriber("/sensor_msgs/Imu", sensor_msgs.Imu._TYPE);

		latch = new CountDownLatch(1);
		displayLatch = new CountDownLatch(1);
		
		SynchronizedFixedThreadPoolManager.spin(new Runnable() {
			@Override
			public void run() {
				if(DEBUG)
					System.out.println("Entering processing loop");
				while(shouldStore) {
					try {
						try {
							latch.await();
						} catch (InterruptedException e) {
							shouldStore = false;
						}
						synchronized(mutex) {
							//if(!imageDiff()) // creates imagel
							//	continue;
							imager = createImage(bufferr);
							imagel = createImage(bufferl);
							drgr = imageDiff(imager);
							drgl = imageDiff(imagel);
							//byte[] bl = convertImage(imagel, drgl);
							//byte[] br = convertImage(imager, drgr);
							//if(DEBUG)
							//	System.out.println("JPG buffers to DB size ="+bl.length+" "+br.length);
							//StereoscopicImageBytes sib = new StereoscopicImageBytes(bufferl, bufferr);
							//StereoscopicImageBytes sib = new StereoscopicImageBytes(bl, br);
							//try {
								//session.store(new Long(System.currentTimeMillis()), new Double(eulers[0]), sib);
								//session.put(sib);
							//} catch (DuplicateKeyException e) {
								// if within 1 ms, rare but occurs
							//}
							if(sequenceNumber%commitRate == 0) {
								//System.out.println("Committing at sequence "+sequenceNumber);
								//session = new RelatrixClient(DATABASE, DATABASE, DATABASE_PORT);
								//session.Commit();
								//session = BigSackAdapter.getBigSackTransactionalHashSet(StereoscopicImageBytes.class);
								if(MAXIMUM > 0 && sequenceNumber >= MAXIMUM) {
									//session.close();
									shouldStore = false;
								}
							}
						}
						displayLatch.countDown();
						displayLatch = new CountDownLatch(1);
					} catch (/*IllegalAccessException |*/ IOException e) {
						System.out.println("Storage failed for sequence number:"+sequenceNumber+" due to:"+e);
						e.printStackTrace();
						shouldStore = false;
					} /*catch (DuplicateKeyException e) {
					System.out.println("Duplicate key at "+System.currentTimeMillis());
				}*/
				}
				
				System.exit(1);
			}
		}, "VIDEORECORDERCLIENT");
		
		main(null); // fire up rest of display pipeline
		
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
				//i                                             nt igreen = (ib.get(i) >> 8 ) & 0x0ff;
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
	
	void initNPU(String modelFile) throws IOException {
		byte[] bmodel = model.load(modelFile);
		long tim = System.currentTimeMillis();
		ctx = model.init(bmodel);
		System.out.println("Init time:"+(System.currentTimeMillis()-tim)+" ms.");
		rknn_sdk_version sdk = model.querySDK(ctx);
		ioNum = model.queryIONumber(ctx);
		System.out.printf("%s %s%n", sdk, ioNum);
		//BufferedImage bimage = Instance.readBufferedImage(args[1]);
		//Instance image = null;
		inputAttrs = new rknn_tensor_attr[ioNum.getN_input()];
		for(int i = 0; i < ioNum.getN_input(); i++) {
			inputAttrs[i] = model.queryInputAttrs(ctx, i);
		//	System.out.println("Tensor input layer "+i+" attributes:");
		//	System.out.println(RKNN.dump_tensor_attr(inputAttrs[i]));
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
			//System.out.println("Tensor output layer "+i+" attributes:");
			//System.out.println(RKNN.dump_tensor_attr(outputAttr));
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
		//System.out.println("Total category labels="+labels.length);
		//System.out.println("Setup time:"+(System.currentTimeMillis()-tim)+" ms.");
	}
	
	detect_result_group imageDiff(Instance image) throws IOException {
		// Set input data, example of setInputs
		model.setInputs(ctx,widthHeightChannel[0],widthHeightChannel[1],widthHeightChannel[2],inputAttrs[0].getType(),inputAttrs[0].getFmt(),image.getRGB888());
		rknn_output[] outputs = model.setOutputs(ioNum.getN_output(), false, wantFloat); // last param is wantFloat, to force output to floating
		long tim = System.currentTimeMillis();
		model.run(ctx);
		System.out.println("Run time:"+(System.currentTimeMillis()-tim)+" ms.");
		System.out.println("Getting outputs...");
		tim = System.currentTimeMillis();
		model.getOutputs(ctx, ioNum.getN_output(), outputs);
		System.out.println("Get outputs time:"+(System.currentTimeMillis()-tim)+" ms.");
		System.out.println("Outputs:"+Arrays.toString(outputs));
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
	
	
	static class PlayerFrame extends JPanel {
		JFrame frame;
		public PlayerFrame() {
			//frame = new JFrame("Player");
			//---
			//displayPanel = new PlayerFrame();
			//frame.getContentPane().add(displayPanel, BorderLayout.CENTER);
			//displayPanel.setVisible(true);
			//---
			//frame.add(this, BorderLayout.CENTER);
			// Remove window title and borders
	        //frame.setUndecorated(true);
	        // Make frame topmost
	        //frame.setAlwaysOnTop(true);
	        // Disable Alt+F4 on Windows
	        //frame.setDefaultCloseOperation(WindowConstants.DO_NOTHING_ON_CLOSE);
	        // Make frame full-screen
	        //frame.setExtendedState(Frame.MAXIMIZED_BOTH);
	        // Display frame
			//this.setVisible(true);
			//frame.pack();
			//frame.setSize(new Dimension(640, 480));
			//frame.setVisible(true);
		}
		private java.awt.Image lastFrame;
		public void setLastFrame(java.awt.Image lf) {
			if( lf == null ) return;
			if( lastFrame == null ) {
				lastFrame = lf;
				return;
			}
			synchronized(lastFrame) {
				lastFrame = lf; 
			} 
		}

		public void paintComponent(Graphics g) {
			super.paintComponent(g);
			if( lastFrame == null )
				return;
			synchronized(lastFrame) {
				g.drawImage(lastFrame, 0, 0, lastFrame.getWidth(this), lastFrame.getHeight(this), this);
				lastFrame.flush();
			}
		}
		
		public void paintPanel(){
		   /*
		    * Use a "Grid Bag Layout" for this panel.  It will be laid
		    * out like this:
		    * Yaw:<actual Yaw> Roll:<actual roll> Pitch:<actual pitch>
		    */
		        GridBagLayout      gbl      = new GridBagLayout();
		        GridBagConstraints gbc = new GridBagConstraints();
		        setLayout(gbl);
		        gbc.anchor=gbc.LAST_LINE_START;


		   /*
		    * Set up the "Yaw" label
		    */
		        gbc.fill=gbc.HORIZONTAL;
		        JLabel xJLabel = new JLabel("Yaw:");
		        gbc.gridx=1;
		        gbc.gridy=1;
		        gbc.weightx=0;
		        gbc.weighty=1;
		        gbl.setConstraints(xJLabel, gbc);
		        add(xJLabel);

		   /*
		    * Set up the "Time" label
		    */
		        JLabel yJLabel = new JLabel("Time:");
		        gbc.fill=gbc.HORIZONTAL;
		        gbc.gridx=2;
		        gbc.gridy=1;
		        gbc.weightx=0;
		        gbc.weighty=1;
		        gbl.setConstraints(yJLabel, gbc);
		        add(yJLabel);

		   /*
		    * Set up the "Pitch" label
		    */
		        JLabel aJLabel = new JLabel("Pitch:");
		        gbc.fill=gbc.HORIZONTAL;
		        gbc.gridx=3;
		        gbc.gridy=1;
		        gbc.weightx=0;
		        gbc.weighty=1;
		        gbl.setConstraints(aJLabel, gbc);
		        add(aJLabel);

		   
		   /*
		    * Set up the text field for the actual X coordinate
		    */
		        xField = new JTextField(8);
		        xField.setEditable(false);
		        xField.setHorizontalAlignment(JTextField.RIGHT);
		        xField.setBorder(null);
		        gbc.fill=gbc.NONE;
		        gbc.gridx=1;
		        gbc.gridy=1;
		        gbc.weightx=1;
		        gbc.weighty=1;
		        gbl.setConstraints(xField, gbc);
		        add(xField);

		   /*
		    * Set up the text field for the actual Y coordinate
		    */
		        yField = new JTextField(18);
		        yField.setEditable(false);
		        yField.setHorizontalAlignment(JTextField.RIGHT);
		        yField.setBorder(null);
		        gbc.fill=gbc.NONE;
		        gbc.gridx=2;
		        gbc.gridy=1;
		        gbc.weightx=1;
		        gbc.weighty=1;
		        gbl.setConstraints(yField, gbc);
		        add(yField);

		   /*
		    * Set up the text field for the actual angle (theta)
		    */
		        aField = new JTextField(8);
		        aField.setEditable(false);
		        aField.setHorizontalAlignment(JTextField.RIGHT);
		        aField.setBorder(null);
		        gbc.fill=gbc.NONE;
		        gbc.gridx=3;
		        gbc.gridy=1;
		        gbc.weightx=1;
		        gbc.weighty=1;
		        gbl.setConstraints(aField, gbc);
		        add(aField);


		   /*
		    * Create formatters for the coordinates and time (2 decimal places)
		    * and the angles (1 decimal place, and the degrees symbol).
		    */
		        dt = DateFormat.getTimeInstance();
		        df = DateFormat.getDateInstance();
		        dfs = new DecimalFormat("+####0.00;-####0.00");
		        da = new DecimalFormat("####0.0\u00b0");
		        das = new DecimalFormat("+####0.0\u00b0;-####0.0\u00b0");

		   /*
		    * Initialize all displayed values to 0, except the angles,
		    * which are set to 90 degrees.
		    */
		        setComputedValues(0.0, 0, 0.0);
		    }


		    /* Convert from radians to degrees, and coerce to range (-180, 180] */
		    private double degree(double a){
		        double d = a*180.0/Math.PI;
		        if(d<0)
		           d = -d;
		        d = d - 360.0*Math.floor(d/360.0);
		        if(a<0)
		           d=360.0-d;
		        if(d>180.0)
		           d-=360;
		        return d;
		    }


		    /**
		     * Display both the true and dead-reckoned positions
		     * (location and direction).
		     */
		    void setComputedValues(double yaw, long time, double pitch)
		    {
		        xField.setText(da.format(yaw));
		        yField.setText(dt.format(time)+" "+df.format(time));
		        aField.setText(da.format(pitch));

		    }

		    /** Formatter for the time. */
		    DateFormat dt,df;

		    /** Formatter for the errors (2 decimal places, allows shows + or -). */
		    DecimalFormat dfs;

		    /** Formatter for the angles (1 decimal place, and the "&deg;" symbol). */
		    DecimalFormat da;

		    /** Formatter for the angle errors (1 decimal place, allows include + or - 
				 * and the "&deg;" symbol). */
		    DecimalFormat das;

		   /** Field for displaying the "true" X coordinate. */
		    JTextField xField;

		    /** Field for displaying the "true" Y coordinate. */
		    JTextField yField;
		    /** Field for displaying the "true" direction. */
		    JTextField aField;

		}

	public static void main(String[] args) {
			
		SwingUtilities.invokeLater(new Runnable() {
			public void run() {
					JFrame frame = new JFrame("Player");
			        displayPanel1 = new PlayerFrame();
			        frame.setLayout(new BorderLayout());
					frame.add(displayPanel1, BorderLayout.EAST);
					displayPanel1.setPreferredSize(new Dimension(640,480));
					displayPanel1.setVisible(true);
			        displayPanel2 = new PlayerFrame();
					frame.add(displayPanel2, BorderLayout.WEST);
					displayPanel2.setPreferredSize(new Dimension(640,480));
					displayPanel2.setVisible(true);
					frame.pack();
					frame.setSize(new Dimension(1320, 520));
					frame.setVisible(true);
			        displayPanel1.paintPanel();
			        displayPanel2.paintPanel();
			    }
		});
			
			while(displayPanel1 == null || displayPanel2 == null)
				try {
					Thread.sleep(1);
				} catch (InterruptedException e) {}
		
			try {
				SynchronizedFixedThreadPoolManager.spin(new Runnable() {
					@Override
					public void run() {
						if(DEBUG)
							System.out.println("Entering display loop");
				        while(true) {
				        	//java.awt.Image dimage=null;
							//try {
								//dimage = queue.takeFirst();
							//} catch (InterruptedException e) {
								//e.printStackTrace();
							//}
							try {
								displayLatch.await();
							} catch (InterruptedException e1) {
								e1.printStackTrace();
							}
				        	if( imagel == null || imager == null) {
								try {
									Thread.sleep(1);
								} catch (InterruptedException e) {}
				        	} else {
				        		synchronized(mutex) {
				        			displayPanel1.setLastFrame((java.awt.Image)imagel.drawDetections(drgl));
				        			displayPanel2.setLastFrame((java.awt.Image)imager.drawDetections(drgr));
				        			displayPanel2.setComputedValues(eulers[0],(long) eulers[1],eulers[2]);
				        			displayPanel1.setComputedValues(eulers[0],(long) eulers[1],eulers[2]); // values from db for time, yaw
				        			//displayPanel.lastFrame = displayPanel.createImage(new MemoryImageSource(newImage.imageWidth
				        			//		, newImage.imageHeight, buffer, 0, newImage.imageWidth));
				        			displayPanel1.invalidate();
				        			displayPanel2.invalidate();
				        			displayPanel1.updateUI();
				        			displayPanel2.updateUI();
				        		}
				        	}
				        }
					}
				});
			} catch( IllegalArgumentException iae) {
				iae.printStackTrace();
				return;
			}
		
		} // run

}

