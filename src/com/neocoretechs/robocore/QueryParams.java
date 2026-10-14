package com.neocoretechs.robocore;

import java.util.Collection;
import java.util.List;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Subscriber;



/**
 * Query param server
 * @author Jonathan Groff (C) NeocoreTechs 2026
 *
 */
public class QueryParams extends AbstractNodeMain {
	private static boolean DEBUG = false;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("subs_queryparams");
	}
	
	@Override
	public void onStart(final ConnectedNode connectedNode) {
			ParameterTree pTree = connectedNode.getParameterTree();
			Collection<GraphName> elems = pTree.getNames();
			Object[] o = elems.toArray();
			for(int i = 0; i < elems.size(); i++) {
				System.out.printf("%d.) %s%s", o[i]);
			}
	};

}
