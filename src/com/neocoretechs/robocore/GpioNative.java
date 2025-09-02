package com.neocoretechs.robocore;

import java.util.Scanner;

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
    	if(args.length < 3) {
    		System.out.println("usage: java GpioNative <chipname> <chip line name> <value to set line to>");
    		System.exit(1);
    	}
    	GpioNative gpio = new GpioNative();
    	int ret = gpio.openChip(args[0]);
    	if(ret < 0)
    		throw new RuntimeException("cant open "+args[0]);
    	// get handle to line struct
    	int lineNum = gpio.findChipLine(args[1]);
    	if(lineNum < 0)
    		throw new RuntimeException("cant find chip line "+args[1]);
    	ret = gpio.lineRequestOutput(lineNum);
    	if(ret < 0)
    		throw new RuntimeException("cant get output for line "+args[1]);
    	ret = gpio.lineSetValue(lineNum, Integer.parseInt(args[2]));
    	if(ret < 0)
    		throw new RuntimeException("cant set line "+args[1]+" to "+args[2]);
    	System.out.println("Success!");
        Scanner scanner = new Scanner(System.in);
        System.out.println("Press Enter to close chip and finish...");
        scanner.nextLine(); // Wait for user input
        scanner.close();
    	// release optional but good practice
    	gpio.lineRelease(lineNum);
    	gpio.closeChip();
    }

}
