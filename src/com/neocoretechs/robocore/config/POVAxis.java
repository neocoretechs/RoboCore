package com.neocoretechs.robocore.config;
/**
 * Defines a POV axis. The position in the jinput array is fixed at 6, with the value 
 * of .25 being returned to indicate the position is set
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2026
 */
public final class POVAxis implements AxisInterface<com.neocoretechs.robocore.config.POVAxis.POVType> {
	private static final long serialVersionUID = 1L;
	private static final String type = "POV";
	private static final String[] nameArray= {"Axis"};
	public enum POVType {
		AxisUp("AxisUp"),
		AxisDown("AxisDown"),
		AxisLeft("AxisLeft"),
		AxisRight("AxisRight");
		String name;
		POVType(String name) { this.name = name;}
		public String val() { return name; }
	};
	
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
		return new int[] {6};
	}
	@Override
	public void setPositionArray(int[] values) {
	}
	@Override
	public POVType matches(float[] axes) {
		if(axes[6] ==.25f) {
			return POVType.AxisUp;
		}
		if(axes[6] == .75f) {
			return POVType.AxisDown;
		}
		if(axes[6] == 1.0f) {
			return POVType.AxisLeft;
		}
		if(axes[6] == .5f) {
			return POVType.AxisRight;
		}
		throw new RuntimeException("Bad value returned from POV axis:"+axes[6]);
	}
}
