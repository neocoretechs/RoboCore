package com.neocoretechs.robocore;

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
import org.ros.namespace.GraphName;
import org.ros.namespace.NameResolver;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeMainExecutor;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.node.topic.Publisher;
import org.ros.internal.loader.CommandLineLoader;

import com.neocoretechs.robocore.machine.bridge.CircularBlockingDeque;

/**
 * Publishes user constructed messages to the Marlinspike MegaPubs controller then onto Marlinspike MegaControl controller.
 * Various functions such as PWM directives on cmd_pwm channel, and directives to generate the reporting 
 * function on cmd_report channel in the Marlinspike code on the Mega board.
 * @author Jonathan Groff (C) NeoCoreTechs 2020
 */
public class MegaPubPubs extends AbstractNodeMain  {
	private static final boolean DEBUG = true;
	private String host;
	private String command = "report";
	private String rptName = "megastatus";
	private InetSocketAddress master;
	private CountDownLatch awaitStart = new CountDownLatch(1);
	std_msgs.UInt32MultiArray pwmmsg = null;
	public CircularBlockingDeque<int[]> pubdataPWM = new CircularBlockingDeque<int[]>(16);
	public CircularBlockingDeque<String> pubdataRPT = new CircularBlockingDeque<String>(16);

	
	public MegaPubPubs(String host, InetSocketAddress master) {
		this.host = host;
		this.master = master;
		NodeConfiguration nc = build();
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, nc);
	    try {
			awaitStart.await();
		} catch (InterruptedException e) {}
	    
	}
	
	public MegaPubPubs(String[] args) {
		CommandLineLoader cl = new CommandLineLoader(Arrays.asList(args));
	    NodeMainExecutor nodeMainExecutor = DefaultNodeMainExecutor.newDefault();
	    nodeMainExecutor.execute(this, cl.build());
	    try {
			awaitStart.await();
		} catch (InterruptedException e) {}
	}
	
	public MegaPubPubs() {}
	
	public GraphName getDefaultNodeName() {
		return GraphName.of("megapub_pubs");
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
	switch(command) {
		case "report":
			//TODO: add additional command line report names and add to queue
			if( remaps.containsKey("__name") ) {
				rptName = remaps.get("__name");
			}
			pubdataRPT.addFirst(rptName);
			break;
		case "pwm":
			// spinup reader that will place PWM commands on queue
			fileReader reader = new fileReader();
			ThreadPoolManager.getInstance().spin(reader, "SYSTEM");
			break;
		default:
			pubdataRPT.addFirst(rptName);
			break;
	}
	//final RosoutLogger log = (Log) connectedNode.getLog();
	
	final Publisher<std_msgs.UInt32MultiArray> pwmpub =
		connectedNode.newPublisher("cmd_pwm", std_msgs.UInt32MultiArray._TYPE);
	
	final Publisher<std_msgs.String> rptpub =
			connectedNode.newPublisher("cmd_report", std_msgs.String._TYPE);

	connectedNode.executeCancellableLoop(new CancellableLoop() {
		@Override
		protected void setup() {
		}

		@Override
		protected void loop() throws InterruptedException {
			switch(command) {
				case "report":
					if( !pubdataRPT.isEmpty() ) {
						std_msgs.String val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.String._TYPE);
						val.setData(pubdataRPT.takeFirst());
						if( DEBUG) System.out.println("Sending report "+val.getData()+", results should appear on StatusAlertSubs console..");
						rptpub.publish(val);
					}
					break;
				case "pwm":
					if( !pubdataPWM.isEmpty() ) {
						int[] pubc = pubdataPWM.takeFirst();
						std_msgs.UInt32MultiArray val32 = connectedNode.getTopicMessageFactory().newFromType(std_msgs.UInt32MultiArray._TYPE);
						if( DEBUG) System.out.println("Set pwm:"+pubc[0]+","+pubc[1]);
						pwmpub.publish(val32);
					}	
					break;
				default:
					if( !pubdataRPT.isEmpty() ) {
						std_msgs.String val = connectedNode.getTopicMessageFactory().newFromType(std_msgs.String._TYPE);
						val.setData(pubdataRPT.takeFirst());
						if( DEBUG) System.out.println("Sending report "+val.getData()+", results should appear on StatusAlertSubs console..");
						rptpub.publish(val);
					}
					break;
			}
			Thread.sleep(10);		
		}

	});

}

class fileReader implements Runnable {
	public volatile boolean shouldRun = true;
	
	@Override
	public void run() {
		while(shouldRun) {
		try {
			FileReader fis = new FileReader("/home/jg/pwms");
			BufferedReader br = new BufferedReader(fis);
			String s = br.readLine();
			br.close();
			fis.close();
			System.out.println(s);
			String left = s.substring(0,s.indexOf(","));
			String right = s.substring(s.indexOf(",")+1);
			System.out.println(left+","+right);
			int l = Integer.parseInt(left,10);
			int r = Integer.parseInt(right,10);
			pubdataPWM.addLast(new int[]{l,r});
			Thread.sleep(5);
		} catch (Exception e) {
			// TODO Auto-generated catch block
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
}
}
