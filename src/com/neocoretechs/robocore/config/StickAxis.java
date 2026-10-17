package com.neocoretechs.robocore.config;
/**
 * Defines a Stick axis. Upon configuration we have to set the position of the stick axis desired.
 * We do this through the constructor.
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2026
 */
public final class StickAxis implements AxisInterface<Integer[]> {
	private static final long serialVersionUID = 1L;
	private static final String type = "Stick";
	private static final String[] nameArray= {"AxisX","AxisY"};
	private int[] positionArray = {0,0};
	public StickAxis(int[] pos) {
		this.positionArray = pos;
	}
	@Override
	public String getType() {
		return type;
	}
	@Override
	public String[] getNameArray() {
		return nameArray;
	}
	@Override
	public int[] getPositionArray() {
		return positionArray;
	}
	@Override
	public void setPositionArray(int[] values) {
		positionArray = values;
	}
	@Override
	public Integer[] matches(float[] axes) {
		Integer[] res = new Integer[2];
		res[0] = -(int)(axes[positionArray[0]]*1000.0f);
		res[1] = -(int)(axes[positionArray[1]]*1000.0f);
		return res;
	}
	@Override
	public Boolean matches(int[] buttons) {
		return false;
	}
}
