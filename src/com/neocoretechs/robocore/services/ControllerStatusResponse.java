package com.neocoretechs.robocore.services;

public interface ControllerStatusResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "com.neocoretechs.robocore.services/ControllerStatusResponse";
  static final java.lang.String _DEFINITION = "string megastatus";
  String getMegastatus();
  void setMegastatus(String value);
}
