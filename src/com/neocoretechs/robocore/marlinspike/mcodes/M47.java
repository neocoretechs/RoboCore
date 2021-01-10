package com.neocoretechs.robocore.marlinspike.mcodes;

import java.util.Map;

import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer;
import com.neocoretechs.robocore.machine.bridge.TopicList;
import com.neocoretechs.robocore.machine.bridge.AsynchDemuxer.topicNames;
/**
 * M47 -Read analog pin P<pin> T<threshold> compare to battery threshold, if below, print battery message.<p/>
 * Unlike other operations, this one will ack with either the return M47/, or a battery message:<br/>
 * 	SERIAL_PGM(MSG_BEGIN);<br/>
 *	SERIAL_PGM(batteryCntrlHdr);<br/>
 *	SERIAL_PGMLN(MSG_DELIMIT);<br/>
 *	SERIAL_PGM("1 ");<br/>
 *	SERIAL_PORT.println(volts);<br/>
 *	SERIAL_PGM(MSG_BEGIN);<br/>
 *	SERIAL_PGM(batteryCntrlHdr);<br/>
 *	SERIAL_PGMLN(MSG_TERMINATE);<br/>
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M47 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M47(AsynchDemuxer asynchDemuxer, Map<String, TopicList> topics) {
		super(asynchDemuxer, topics, topicNames.M47.val());
	}
}
