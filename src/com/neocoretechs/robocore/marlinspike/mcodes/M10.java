package com.neocoretechs.robocore.marlinspike.mcodes;


import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * Dynamically allocate a controller to a control slot. The slot parameter is used to refer
 * to the dynamically allocated controller in other M codes that relate to motor control functions.
 * The M10 code merely creates the instance of the proper controller and assigns the slot. Other M codes
 * refer to the slot and provide further configuration. when creating new type of controllers, this is the code
 * that can be expanded to instantiate those controllers
 * M10 Z<controller slot> T<controller type>
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M10 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M10(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.M10.val());
	}

}
