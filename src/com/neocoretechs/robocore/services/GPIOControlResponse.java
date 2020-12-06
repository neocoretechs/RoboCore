package com.neocoretechs.robocore.services;

public interface GPIOControlResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "com.neocoretechs.robocore.services/GPIOControlResponse";
  static final java.lang.String _DEFINITION = "string status";
  String getStatus();
  void setStatus(String value);
}
