package com.neocoretechs.robocore;
import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.LineUnavailableException;
import javax.sound.sampled.SourceDataLine;


public class GenerateTone {
	 /** Generates a tone.
	  @param hz Base frequency (neglecting harmonic) of the tone in cycles per second
	  @param msecs The number of milliseconds to play the tone.
	  @param volume Volume, form 0 (mute) to 100 (max).
	  @param addHarmonic Whether to add an harmonic, one octave up. */
	  public static void generateTone(int hz,int msecs, int volume, boolean addHarmonic)
	    throws LineUnavailableException {

	    float frequency = 44100;
	    byte[] buf;
	    AudioFormat af;
	    if (addHarmonic) {
	      buf = new byte[2];
	      af = new AudioFormat(frequency,8,2,true,false);
	    } else {
	      buf = new byte[1];
	      af = new AudioFormat(frequency,8,1,true,false);
	    }
	    SourceDataLine sdl = AudioSystem.getSourceDataLine(af);
	    sdl = AudioSystem.getSourceDataLine(af);
	    sdl.open(af);
	    sdl.start();
	    for(int i=0; i<msecs*frequency/1000; i++){
	      double angle = i/(frequency/hz)*2.0*Math.PI;
	      buf[0]=(byte)(Math.sin(angle)*volume);

	      if(addHarmonic) {
	        double angle2 = (i)/(frequency/hz)*2.0*Math.PI;
	        buf[1]=(byte)(Math.sin(2*angle2)*volume*0.6);
	        sdl.write(buf,0,2);
	      } else {
	        sdl.write(buf,0,1);
	      }
	    }
	    sdl.drain();
	    sdl.stop();
	    sdl.close();
	  }

	public static void main(String[] args) throws LineUnavailableException {
		
		for(int i = 0; i < 15; i++) {
		GenerateTone.generateTone(1500, 50, 100, true);
		try {
			Thread.sleep(10);
		} catch (InterruptedException e) {
	
			e.printStackTrace();
		}
		}
	}

}
