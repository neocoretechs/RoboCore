package com.neocoretechs.robocore.services;

import std_msgs.UInt32MultiArray;

public interface GPIOControlRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "com.neocoretechs.robocore.services/GPIOControlRequest";
  static final java.lang.String _DEFINITION = "std_msgs/UInt32MultiArray gpio\n";
  UInt32MultiArray getGpio();
  void setGpio(UInt32MultiArray value);

}
