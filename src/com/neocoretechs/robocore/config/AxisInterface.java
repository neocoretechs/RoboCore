package com.neocoretechs.robocore.config;

import java.io.Serializable;

public interface AxisInterface<T> extends Serializable {
	public String getType();
	public String[] getNameArray();
	public int[] getPositionArray();
	public void setPositionArray(int[] values);
	public boolean isAxis();
	public boolean isButton();
	public T matches(float[] axes);
	public T matches(int[] buttons);
}
