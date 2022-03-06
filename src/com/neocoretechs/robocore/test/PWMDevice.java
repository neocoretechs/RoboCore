package com.neocoretechs.robocore.test;

import java.io.FileDescriptor;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.RandomAccessFile;

public class PWMDevice {
	static {
		try {
			pwm = new RandomAccessFile("/sys/devices/platform/pwm-ctrl/duty0","rw");
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	static RandomAccessFile pwm;
	public static void main(String[] args) throws IOException {
		for(int i = 0; i < args.length; i++) {
			pwm.writeBytes(args[i]);
			pwm.seek(0);
			String pwmr = pwm.readLine();
			System.out.println(pwmr);
		}
		//FileDescriptor fd = pwm.getFD();

	}

}
