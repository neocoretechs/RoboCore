package com.neocoretechs.robocore.config;

public final class Button implements AxisInterface<Boolean> {
	private static final long serialVersionUID = 1L;
	private static final String type = "Button";
	private static final String[] nameArray= {"Button"};
	private int[] positionArray;
	public Button(int[] is) {
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
	public Boolean matches(float[] axes) {
		return axes[positionArray[0]] != 0;
	}
}
