package com.neocoretechs.robocore.config;

import com.neocoretechs.robocore.config.POVAxis.POVType;

/**
 * Defines a POV axis. The position in the jinput array is fixed at 6, with the value 
 * of .25 being returned to indicate the position is set
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2026
 */
public final class POVAxis implements AxisInterface<Integer[]> {
	private static final long serialVersionUID = 1L;
	private static final String type = "POV";
	private static final String[] nameArray= {"Axis"};
	public enum POVType {
		AxisUp("AxisUp"),
		AxisDown("AxisDown"),
		AxisLeft("AxisLeft"),
		AxisRight("AxisRight"),
		AxisUpperLeft("AxisUpperLeft"),
		AxisUpperRight("AxisUpperRight"),
		AxisLowerLeft("AxisLowerLeft"),
		AxisLowerRight("AxisLowerRight"),
		AxisOff("AxisOff");
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
	public POVType type(float[] axes) {
		if(axes[6] == 0.0f) {
			return POVType.AxisOff;
		}
		if(axes[6] == .25f) {
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
		if(axes[6] == .125f) {
			return POVType.AxisUpperLeft;
		}
		if(axes[6] == .375f) {
			return POVType.AxisUpperRight;
		}
		if(axes[6] == .875f) {
			return POVType.AxisLowerLeft;
		}
		if(axes[6] == .625f) {
			return POVType.AxisLowerRight;
		}
		throw new RuntimeException("Bad value returned from POV axis:"+axes[6]);
	}
	@Override
	public Integer[] matches(float[] axes) {
		POVType pov = type(axes);
		Integer[] ret = new Integer[1];
		switch(pov) {
		case POVType.AxisUp:
			ret[0] = 1000;
			break;
		case POVType.AxisDown:
			ret[0] = -1000;
			break;
		case POVType.AxisLeft:
		case POVType.AxisRight:
			ret[0] = 0;
			break;
		case POVType.AxisUpperLeft:
			ret[0] = 500;
			break;
		case POVType.AxisUpperRight:
			ret[0] = 250;
			break;
		case POVType.AxisLowerLeft:
			ret[0] = -500;
			break;
		case POVType.AxisLowerRight:
			ret[0] = -250;
			break;
		case POVType.AxisOff:
			ret[0] = 0;
			break;
		default:
			throw new RuntimeException("Undefined POV state");
		}
		return ret;
	}
	@Override
	public Integer[] matches(int[] buttons) {
		return new Integer[0];
	}
	@Override
	public boolean isAxis() {
		return true;
	}
	@Override
	public boolean isButton() {
		return false;
	}
}
