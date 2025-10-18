package com.neocoretechs.robocore.video;

import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.Serializable;
import java.util.Objects;

import javax.imageio.ImageIO;

import com.neocoretechs.rknn4j.image.Instance;

public final class TimedImage implements Comparable, Serializable {
	private static final long serialVersionUID = 1L;
    transient byte[] prevBufferl = null;
    transient byte[] prevBufferr = null;
	final static float IMAGE_DIFFERENCE_PCT = .15f;
	long sequence;
	long time;
	public byte[] leftImage;
	public byte[] rightImage;
	double eulersCorrelated[] = new double[]{0.0,0.0,0.0};
	float rangeCorrelated = 0f;
	transient stereo_msgs.StereoImage stereoImage;
	boolean skipSegLeft = false;
	boolean skipSegRight = false;
	public TimedImage() {}
	public TimedImage(stereo_msgs.StereoImage stereoImage, long sequence) {
		this.stereoImage = stereoImage;
		this.leftImage = stereoImage.getData().array();
		this.rightImage = stereoImage.getData2().array();
		this.time = System.currentTimeMillis();
		this.sequence = sequence;
			try {
				Instance rimage = createImage(rightImage);
				if(prevBufferr == null)
					prevBufferr = rimage.getImageByteArray();
				if(rimage.shouldSegment(prevBufferr, IMAGE_DIFFERENCE_PCT)) {
					prevBufferr = rimage.getImageByteArray();
					skipSegRight = false;
				} else
					skipSegRight = true;

				Instance limage = createImage(leftImage);
				if(prevBufferl == null)
					prevBufferl = limage.getImageByteArray();
				if(limage.shouldSegment(prevBufferl, IMAGE_DIFFERENCE_PCT)) {
					prevBufferl = limage.getImageByteArray();
					skipSegLeft = false;
				} else
					skipSegLeft = true;
			} catch (IOException e) {
				e.printStackTrace();
			}
	}
	
	@Override
	public int compareTo(Object o) {
		if(time > ((TimedImage)o).time)
			return 1;
		if(time < ((TimedImage)o).time)
			return -1;
		return 0;
	}
	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + Objects.hash(time);
		return result;
	}
	/**
	 * We assume we tossed out anything with zero in both detections
	 * If the left number matches the right number and each occurrence of name in left and right match, they are 'equal'
	 */
	@Override
	public boolean equals(Object obj) {
		if (this == obj) {
			return true;
		}
		if (!(obj instanceof TimedImage)) {
			return false;
		}
		return (this.time == ((TimedImage)obj).time);
	}
	/**
	 * Generate Instance from raw JPEG image buffer with RGA
	 * @param imgBuff
	 * @return
	 * @throws IOException
	 */
	Instance createImage(byte[] imgBuff) throws IOException {
		InputStream is = new ByteArrayInputStream(imgBuff);
		BufferedImage image = ImageIO.read(is);
		return new Instance("tmp", image, "tmp");
	}
}

