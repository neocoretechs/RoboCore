package com.neocoretechs.robocore.test;

import java.io.FileDescriptor;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.RandomAccessFile;

public class PWMDevice {

	public static void main(String[] args) throws IOException {
		RandomAccessFile pwm = new RandomAccessFile("/sys/devices/platform/pwm-ctrl/duty0","rw");
		pwm.writeBytes("512");
		//FileDescriptor fd = pwm.getFD();

	}

}
