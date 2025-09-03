package com.neocoretechs.robocore.marlinspike.mcodes;

import com.neocoretechs.robocore.marlinspike.AbstractBasicResponse;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer;
import com.neocoretechs.robocore.marlinspike.AsynchDemuxer.topicNames;
/**
* Set Delay HBridge PWM motor driver, map pin to channel, this will check to prevent free running motors during inactivity
* For a PWM motor control subsequent G5 commands are affected here.
* and then D a direction pin that determines forward/backward , then E, the default value of the direction pin.
* The D and E pins determine whether a HIGH or LOW determines FORWARD/BACK on the given controller channel. This is
* to compensate for 'backward' motors mounted left/right, or differences in controller design. Using a combination of
* these 2 parameters you can tune any controller/motor setup properly for forward/back.
*  Finally, W<encoder pin>  to receive hall wheel sensor signals and
* optionally PWM timer setup [R<resolution 8,9,10 bits>] [X<prescale 0-7>].
* The Timer mode (0-3) is preset to 2 in the individual driver. Page 129 in datasheet. Technically we are using a 'non PWM'
* where the 'compare output mode' is defined by 3 operating modes. Since we are unifying all the timers to use all available PWM
* pins, the common mode among them all is the 'non PWM', within which the 3 available operating modes can be chosen from.
* There are essentially three main operating modes:
* 0 - Stop
* 1 - Toggle on compare match
* 2 - Clear on match
* 3 - Set on match
* For motor operation and general purpose PWM, mode 2 the most universally applicable.
* M3 [Z<slot>] P<pin> C<channel> D<direction pin> E<default dir> W<encoder pin> [F freq] [G duty]
* @author Jonathan Groff (C) NeoCoreTechs 2020,2021
*
*/
public class M16 extends AbstractBasicResponse {
	private boolean DEBUG;
	public M16(AsynchDemuxer asynchDemuxer) {
		super(asynchDemuxer, topicNames.M16.val());
	}
}
