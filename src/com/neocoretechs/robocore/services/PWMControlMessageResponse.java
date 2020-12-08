package com.neocoretechs.robocore.services;

import java.io.Serializable;

public class PWMControlMessageResponse implements org.ros.internal.message.Message, Serializable {
  private static final long serialVersionUID = 1L;
  static final java.lang.String _TYPE = "com.neocoretechs.robocore.services/PWMControlMessageResponse";
  static final java.lang.String _DEFINITION = "string data\n";
  public PWMControlMessageResponse() {}
  private java.lang.String data;
  public java.lang.String getData() { return data; }
  public void setData(java.lang.String value) { data = value; }
}
