package com.neocoretechs.robocore;

public class GpioNative {
    static {
        System.loadLibrary("gpiodjni"); // Your compiled .so
    }

    public native int openChip(String chipName);
    public native int findChipLine(String lineNum);
    public native int lineRequestInput(int lineNum);
    public native int lineRequestOutput(int lineNum);
    public native int lineRequestRisingEdgeEvents(int lineNum);
    public native int lineGetValue(int LineNum);
    public native int lineSetValue(int lineNum, int value);
    public native int lineEventWait(int lineNum);
    public native int lineEventRead(int lineNum);
    public native int lineRelease(int lineNum);
    public native void closeChip();

    public static void main(String[] args) {
    	GpioNative gpio = new GpioNative();
    	gpio.openChip(args[0]);
    	// get handle to line struct
    	int lineNum = gpio.findChipLine(args[1]);
    	gpio.lineRequestInput(lineNum);
    	System.out.println(gpio.lineGetValue(lineNum));
    	// release optional but good practice
    	gpio.lineRelease(lineNum);
    	gpio.closeChip();
    }

}
