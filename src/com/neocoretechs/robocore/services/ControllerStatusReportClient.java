package com.neocoretechs.robocore.services;

import java.net.InetSocketAddress;
import java.util.Arrays;
import java.util.Map;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

import org.ros.exception.RemoteException;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeMainExecutor;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.internal.loader.CommandLineLoader;

import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;

/**
 * Publishes user constructed messages to the Marlinspike MegaPubs controller then onto Marlinspike MegaControl controller.
 * directives to generate the reporting 
 * function on cmd_report channel in the Marlinspike code on the Mega board.
 * Commands for reports on status to be send to the Marlinspike via the reporting service 
 * are formatted as followed on the command line:<p/>
 * _service_uri:=cmd_report <br/>
 * _service_uri:=robo_status <br/>
 * 
 * for service_uri:=cmd_report
 * __command:=report __name:=id <br/>
 * __command:=report __name:=status <br/>
 * @author Jonathan Groff (C) NeoCoreTechs 2020
 */
public class ControllerStatusReportClient extends AbstractNodeMain  {
	private static final boolean DEBUG = false;
	private String host;
	private String command = "report";
	private String rptName = "status"; // default report name
	private String service_uri = "cmd_report";
	private InetSocketAddress master;
	private CountDownLatch awaitStart = new CountDownLatch(1);

	public CircularBlockingDeque<String> pubdataRPT = new CircularBlockingDeque<String>(16);

	
	public ControllerStatusReportClient(String[] args) {
		CommandLineLoader cl = new CommandLineLoader(Arrays.asList(args));
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, cl.build());
	    try {
			awaitStart.await();
		} catch (InterruptedException e) {}
	}
	
	public ControllerStatusReportClient() {}
	
	public GraphName getDefaultNodeName() {
		return GraphName.of("megastatusclient");
	}


@Override
public void onStart(final ConnectedNode connectedNode) {
	Map<String, String> remaps = connectedNode.getNodeConfiguration().getCommandLineLoader().getSpecialRemappings();
	if( remaps.containsKey("__service_uri") ) {
		service_uri = remaps.get("__service_uri");
	}
	if( remaps.containsKey("__command") ) {
		command = remaps.get("__command");
	}
	switch(command) {
		case "reset":
			pubdataRPT.addFirst("reset");
			break;
		case "report":
			//TODO: add additional command line report names and add to queue
			if( remaps.containsKey("__name") ) {
				rptName = remaps.get("__name");
			}
			pubdataRPT.addFirst(rptName);
			break;
		default:
			pubdataRPT.addFirst(rptName);
			break;
	}
	ServiceClient<ControllerStatusMessageRequest, ControllerStatusMessageResponse> rptsvc = null;
	//final RosoutLogger log = (Log) connectedNode.getLog();
	try {
		rptsvc = connectedNode.newServiceClient(service_uri, ControllerStatusMessage._TYPE);
	} catch (Exception e) {
		e.printStackTrace();
	}
      //serviceClient = connectedNode.newServiceClient(SERVICE_NAME, test_ros.AddTwoInts._TYPE);
	// tell the waiting constructors that we have registered service clients
	awaitStart.countDown();
	awaitStart = new CountDownLatch(1);
	ControllerStatusMessageRequest request = rptsvc.newMessage();
	while( !pubdataRPT.isEmpty() ) {
			try {
				request.setData(pubdataRPT.takeFirst());
				if( DEBUG) 
					System.out.println("Sending report "+request.getData()+", results should appear on StatusAlertSubs console..");
				try {
					rptsvc.connect(connectedNode.lookupServiceUri(GraphName.of(service_uri)));
				} catch (Exception e1) {
					e1.printStackTrace();
					break;
				}
				rptsvc.call(request, new ServiceResponseListener<ControllerStatusMessageResponse>() {
					     @Override
					     public void onSuccess(ControllerStatusMessageResponse response) {
					    	 System.out.println("Successful Response:"+response);
					    	 System.out.println(response.getData());
					    	 awaitStart.countDown();
					     }
					     @Override
					     public void onFailure(RemoteException e) {
					    	 System.out.println("FAILURE Response:"+e);
					    	 throw new RuntimeException(e);
					     }
				});
			} catch (InterruptedException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
				break;
			}
	}

	try {
		awaitStart.await(10, TimeUnit.SECONDS);
	} catch (InterruptedException e) {
		e.printStackTrace();
	}
} 

}

	/**
	 * Get the hardware type, revision, and serial from /proc/cpuinfo (linux only of course).
	 * @return 3 element string array of hardware type, revision, and serial
	 * @throws IOException If the format is janky
	 
	public static String[] readInfo() throws IOException {
		FileReader fr = new FileReader("/proc/cpuinfo");
		CharBuffer barg = CharBuffer.allocate(2048);
		while( fr.read(barg) != -1);
		fr.close();
		String bargs = new String(barg.array());
		//
		int hardPos = bargs.indexOf("Hardware");
		if( hardPos == -1)
			throw new IOException("Can't find Hardware type in cpuinfo");
		int colPos = bargs.indexOf(':',hardPos)+1;
		if( colPos == -1) {
			throw new IOException("Can't find Hardware type in cpuinfo");
		}
		String bhard = bargs.substring(colPos+1);
		//
		int revPos = bargs.indexOf("Revision");
		if( revPos == -1)
			throw new IOException("Can't find Hardware revision in cpuinfo");
		colPos = bargs.indexOf(':',hardPos)+1;
		if( colPos == -1) {
			throw new IOException("Can't find Hardware revision in cpuinfo");
		}
		String brev = bargs.substring(colPos+1);
		//
		// May not have serial, Odroid C2 does not
		String bser = "000000000000000";
		int serPos = bargs.indexOf("Serial");
		if( serPos != -1) {
			colPos = bargs.indexOf(':',serPos)+1;
			if( colPos != -1) {
				bser = bargs.substring(colPos+1);
			}
		}
		//
		return new String[]{bhard, brev, bser};
	
	}
	*/
