package com.neocoretechs.robocore.config;

public final class POVAxisLeft implements AxisInterface<Boolean> {
	private static final long serialVersionUID = 1L;
	private static final String type = "POVAxisLeft";
	private static final String[] nameArray= {"Axis"};
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
		return null;
	}
	@Override
	public void setPositionArray(int[] values) {
	}
	@Override
	public Boolean matches(float[] axes) {
		return axes[6] == 1.0f;
	}
}
