package com.neocoretechs.robocore.marlinspike.mcodes;

import java.io.Serializable;

import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
 * M11 [Z&lt;slot&gt;] C&lt;channel&gt; [D&lt;duration&gt;] [X&lt;duration&gt;] - Set maximum cycle duration for given channel. If X, slot is PWM
 * @author Jonathan Groff (C) NeoCoreTechs 2020,2021
 *
 */
public class M11 extends AbstractBasicResponse  implements Serializable {
	private boolean DEBUG;
	public M11(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M11.val());
	}
}
