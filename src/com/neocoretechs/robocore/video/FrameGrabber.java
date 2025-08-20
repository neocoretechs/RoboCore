package com.neocoretechs.robocore.video;
public class FrameGrabber {
    static {
        System.loadLibrary("libcamera_framegrabber");
    }

    public native long startCapture(boolean flip);
    public native byte[][] getStereoJPEGFrames(long handle);
    public native void cleanup(long handle);

    public static void main(String[] args) {
        FrameGrabber cam1 = new FrameGrabber();
        long handle1 = cam1.startCapture(false);
	for(int i = 0; i < 100; i++) {
        byte[][] jpegFrame = cam1.getStereoJPEGFrames(handle1);
	if(jpegFrame[0] != null)
	System.out.println(i+"="+"cam1 size:"+jpegFrame[0].length);
	if(jpegFrame[1] != null)
	System.out.println(i+"="+"camr size:"+jpegFrame[1].length);
	}
        // Process the JPEG frame as needed
	cam1.cleanup(handle1);
    }
}
