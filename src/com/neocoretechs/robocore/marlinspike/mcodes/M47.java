package com.neocoretechs.robocore.marlinspike.mcodes;


import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * M47 -Read analog pin P<pin> T<threshold> compare to battery threshold, if below, print battery message.<p>
 * Unlike other operations, this one will ack with either the return M47/, or a battery message:<br>
 * <pre>
 * 	SERIAL(MSG_BEGIN);
 *	SERIAL(batteryCntrlHdr);
 *	SERIAL(MSG_DELIMIT);
 *	SERIAL(volts);
 *	SERIAL(MSG_BEGIN);
 *	SERIAL(batteryCntrlHdr);
 *	SERIAL(MSG_TERMINATE);
 *</pre>
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M47 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M47(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M47.val());
	}
}
