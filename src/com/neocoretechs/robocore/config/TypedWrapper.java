package com.neocoretechs.robocore.config;

import java.util.Map;
/**
 * The TypedWrapper has the property name as string and property value as value for each property
 * indexed as an array type in the properties file. The reason we need a typedwrapper is due to the
 * way types are handled at runtime. We need a concrete type of String as key and object as Object to be declared.<p/>
 * As each property is parsed from the config, it is placed in its proper wrapper and placed in the proper array in the Robot object. 
 * For instance:<p/>
 * Config as follows by LUN, logical unit number:<p/>
 * LUN[0].Name:LeftWheel <br/>
 * LUN[0].NodeName:Control1 <br/>
 * LUN[0].Controller:/dev/ttyACM0 <br/>
 * LUN[0].Type:H-Bridge <br/>
 * LUN[0].Slot:0 <br/>
 * LUN[0].SignalPin0:8 <br/>
 * LUN[0].Channel:1 <br/>
 * LUN[0].EnablePin:24 <br/>
 * LUN[0].Direction:0 <br/>
 * LUN[0].EncoderPin:68 <br/>
 * LUN[0].MinValue:-1000 <br/>
 * LUN[0].MaxValue:1000 <br/>
 * PID[0].MotorPIDRate:1 <br/>
 * PID[0].MotorKp:1.0 <br/>
 * PID[0].MotorKd:1.0 <br/>
 * PID[0].MotorKi:1.0 <br/>
 * PID[0].MotorKo:1.0 <br/>
 * AXIS[0].AxisType:Stick <br/>
 * AXIS[0].AxisX:0 <br/>
 * AXIS[0].AxisY:2 <br/>
 * Also, BUTTON, WHEEL, etc as needed.<p/>
 * So when the GlobalConfigs are parsed in {@code Robot} you get the following set of constructs:<p/>
 * 	private TypedWrapper[] LUN; <br/>
 *	private TypedWrapper[] WHEEL; <br/>
 *	private TypedWrapper[] PID; <br/>
 *	private TypedWrapper[] AXIS; <br/>
 *	private TypedWrapper[] BUTTON; <br/>
 * And the resulting TypedWrapper is a ConcurrentHashMap collection with the elements having key,value pairs like:<p/>
 * LUN[0]: <br/>
 * Name, LeftWheel <br/>
 * Slot, 0 <br/>
 * AXIS[0]: <p/>
 * AxisType, Stick <br/>
 * AxisX, 0 <br/>
 * and so on..
 * @author Jonathan Groff (C) NeoCoreTechs 2021
 *
 */
public class TypedWrapper extends java.util.concurrent.ConcurrentHashMap<String, Object> {
	private static final long serialVersionUID = 1L;
	public TypedWrapper(Object object) {
		this.putAll((Map<? extends String, ? extends Object>) object);
	}
}
