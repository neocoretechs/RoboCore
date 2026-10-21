package com.neocoretechs.robocore.config;
/**
 * Defines a Button. Upon configuration we have to set the position of the button desired.
 * We do this through the constructor. Though a button is a button, and not an axis, we define it in properties as
 * AxisType.Button:(buttontype) with button type being one of:
 * <pre>
 * 		Back,
 *		Home,
 *		LeftStick,
 *		RightStick,
 *		Yellow,
 *		Red,
 *		Green,
 *		Blue,
 *		LeftShoulder,
 *		RightShoulder
 *		</pre>
 * @author Jonathan Groff Copyright (C) NeoCoreTechs 2026
 */
public final class Button implements AxisInterface<Boolean> {
	private static final long serialVersionUID = 1L;
	private static final String type = "Button";
	private static final String[] nameArray= {"Button"};
	private int[] positionArray;
	public enum ButtonType {
		Back("Back"),
		Home("Home"),
		LeftStick("LeftStick"),
		RightStick("RightStick"),
		Yellow("Yellow"),
		Red("Red"),
		Green("Green"),
		Blue("Blue"),
		LeftShoulder("LeftShoulder"),
		RightShoulder("RightShoulder");
		String name;
		ButtonType(String name) { this.name = name;}
		public String val() { return name; }
	}
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
	@Override
	public Boolean matches(int[] buttons) {
		return buttons[positionArray[0]] != 0;
	}
}
