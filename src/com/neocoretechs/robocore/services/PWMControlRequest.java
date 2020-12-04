package com.neocoretechs.robocore.services;

import std_msgs.UInt32MultiArray;

public interface PWMControlRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "com.neocoretechs.robocore.reportservices/PWMControlRequest";
  static final java.lang.String _DEFINITION = "UInt32MultiArray pwm\n";
  UInt32MultiArray getPwm();
  void setPwm(UInt32MultiArray value);

}
