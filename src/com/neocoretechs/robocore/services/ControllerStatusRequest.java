package com.neocoretechs.robocore.services;

public interface ControllerStatusRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "com.neocoretechs.robocore.services/ControllerStatusRequest";
  static final java.lang.String _DEFINITION = "string megastatus\n";
  String getMegastatus();
  void setMegastatus(String value);

}
