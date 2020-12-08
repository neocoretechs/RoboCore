package com.neocoretechs.robocore.services;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Map;
import java.util.concurrent.CountDownLatch;

import org.ros.concurrent.CancellableLoop;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Publisher;
import org.ros.internal.loader.CommandLineLoader;

import com.neocoretechs.robocore.RosArrayUtilities;
import com.neocoretechs.robocore.ThreadPoolManager;
import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;

/**
 * Publishes user constructed messages to the Marlinspike MegaPubs controller then onto Marlinspike MegaControl controller.
 * functions as GPIO directives on cmd_gpio channel,
 * in the Marlinspike code on the Mega board.
 * @author Jonathan Groff (C) NeoCoreTechs 2020
 */
public class GPIOControlClient extends AbstractNodeMain  {
	private static final boolean DEBUG = true;
	private String host;
	private String command = "gpio";
	private InetSocketAddress master;
	private CountDownLatch awaitStart = new CountDownLatch(1);
	std_msgs.UInt32MultiArray gpiomsg = null;
	public CircularBlockingDeque<Integer> pubdataGPIO = new CircularBlockingDeque<Integer>(16);
	private String pin;
	private String value;


	
	public GPIOControlClient(String host, InetSocketAddress master) {
		this.host = host;
		this.master = master;
		NodeConfiguration nc = build();
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, nc);
	    try {
			awaitStart.await();
		} catch (InterruptedException e) {}
	    
	}
	
	public GPIOControlClient(String[] args) {
		CommandLineLoader cl = new CommandLineLoader(Arrays.asList(args));
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, cl.build());
	    try {
			awaitStart.await();
		} catch (InterruptedException e) {}
	}
	
	public GPIOControlClient() {}
	
	public GraphName getDefaultNodeName() {
		return GraphName.of("gpiocontrolclient");
	}

/**
 * Create NodeConfiguration 
 * @throws URISyntaxException 
 */
public NodeConfiguration build()  {
  //NodeConfiguration nodeConfiguration = NodeConfiguration.copyOf(Core.getInstance().nodeConfiguration);
  NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(host, master);
  nodeConfiguration.setParentResolver(NameResolver.newFromNamespace("/"));
  nodeConfiguration.setRosRoot(null); // currently unused
  nodeConfiguration.setRosPackagePath(new ArrayList<File>());
  nodeConfiguration.setMasterUri(master);
  nodeConfiguration.setNodeName(getDefaultNodeName());
  return nodeConfiguration;
}

@Override
public void onStart(final ConnectedNode connectedNode) {
	Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();	
	if( remaps.containsKey("__command") ) {
		command = remaps.get("__command");
	}
	// Based on value of 'command' parameter as 'pwm' or 'direct' open the file and read the pwm values
	// or take 2 values from the command line from "__pin" and "__value" and place those entries onto the
	// queue
	switch(command) {
		case "gpio":
			// spinup reader that will place PWM commands on queue
			fileReader<Integer> reader = new fileReader<Integer>(pubdataGPIO,"/home/pi/gpio", Integer.class);
			ThreadPoolManager.getInstance().spin(reader, "SYSTEM");
			break;
		case "direct":
			if( remaps.containsKey("__pin") ) {
				pin = remaps.get("__pin");
				if( remaps.containsKey("__value") ) {
					value = remaps.get("__value");
					pubdataGPIO.addLast(Integer.parseInt(pin));
					pubdataGPIO.addLast(Integer.parseInt(value));
				}
			}

		default:
			break;
	}
	
	try {
		ServiceClient<GPIOControlMessageRequest, GPIOControlMessageResponse> gpiosvc = null;
		//final RosoutLogger log = (Log) connectedNode.getLog();
		gpiosvc = connectedNode.newServiceClient("cmd_gpio", GPIOControlMessage._TYPE);

      //serviceClient = connectedNode.newServiceClient(SERVICE_NAME, test_ros.AddTwoInts._TYPE);
	// tell the waiting constructors that we have registered service clients
	awaitStart.countDown();
	
	switch(command) {

	case "gpio":
		if( !pubdataGPIO.isEmpty() ) {
			ArrayList<Integer> gpioOut = new ArrayList<Integer>();
			try {
				gpioOut.add(pubdataGPIO.takeFirst());
				gpioOut.add(pubdataGPIO.takeFirst());
			} catch (InterruptedException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
			GPIOControlMessageRequest request = gpiosvc.newMessage();
			request.setData(RosArrayUtilities.setupUInt32Array(connectedNode, gpioOut, "2DInt"));
			if( DEBUG) System.out.println("Set pwm:"+request.getData().getData()[0]+","+request.getData().getData()[1]);
			gpiosvc.call(request, new ServiceResponseListener<GPIOControlMessageResponse>() {
				@Override
				public void onFailure(RemoteException e) {
					throw new RuntimeException(e);		
				}
				@Override
				public void onSuccess(GPIOControlMessageResponse arg0) {
					System.out.println(arg0.getData());	
				}
			});
		}
		break;
		default:
			break;
	}
	} catch (Exception e) {
		e.printStackTrace();
	} 
}
}
