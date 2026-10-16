package com.neocoretechs.robocore.config;

public final class TriggerAxis implements AxisInterface<Integer> {
	private static final long serialVersionUID = 1L;
	private static final String type = "Trigger";
	private static final String[] nameArray= {"Axis"};
	private int[] positionArray = {0};
	public TriggerAxis(int[] is) {
		positionArray = is;
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
	public Integer matches(float[] axes) {
		return (int)axes[positionArray[0]]*1000;
	}
}
