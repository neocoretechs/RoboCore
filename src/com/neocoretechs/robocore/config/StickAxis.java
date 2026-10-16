package com.neocoretechs.robocore.config;

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
		res[0] = -((int)axes[positionArray[0]]*1000);
		res[1] = -((int)axes[positionArray[1]]*1000);
		return res;
	}
}
