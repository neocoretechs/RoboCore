package com.neocoretechs.robocore.services;

import java.io.Serializable;

import std_msgs.UInt32MultiArray;

public class PWMControlMessageRequest implements org.ros.internal.message.Message, Serializable {
  private static final long serialVersionUID = 1L;
  static final java.lang.String _TYPE = "com.neocoretechs.robocore.services/PWMControlMessageRequest";
  static final java.lang.String _DEFINITION = "std_msgs/UInt32MultiArray data\n";
  public PWMControlMessageRequest() {}
  private std_msgs.UInt32MultiArray data;
  public std_msgs.UInt32MultiArray getData() { return data; }
  public void setData(std_msgs.UInt32MultiArray value) { data=value; }
}
