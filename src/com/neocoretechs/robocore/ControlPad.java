package com.neocoretechs.robocore;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;
import java.awt.image.BufferedImage;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.UnknownHostException;
import java.util.EnumMap;
import java.util.Map;

//import javafx.scene.input.KeyCode;

import javax.swing.*;

@SuppressWarnings("serial")
public class ControlPad extends JPanel {
   enum Dir {
      LEFT("Left", KeyEvent.VK_LEFT, -1, 0),
      RIGHT("Right", KeyEvent.VK_RIGHT, 1, 0),
      UP("Up", KeyEvent.VK_UP, 0, -1),
      DOWN("Down", KeyEvent.VK_DOWN, 0, 1),
      KILL("Kill", KeyEvent.VK_K, 0, 0),
      LIFTUP("LiftUp", KeyEvent.VK_U, 0, 0),
      LIFTDOWN("LiftDown", KeyEvent.VK_D, 0, 0);

      private String name;
      private int keyCode;
      private int deltaX;
      private int deltaY;
      private Dir(String name, int keyCode, int deltaX, int deltaY) {
         this.name = name;
         this.keyCode = keyCode;
         this.deltaX = deltaX;
         this.deltaY = deltaY;
      }
      public String getName() {
         return name;
      }
      public int getKeyCode() {
         return keyCode;
      }
      public int getDeltaX() {
         return deltaX;
      }
      public int getDeltaY() {
         return deltaY;
      }      
   }
   public static final int TIMER_DELAY = 10;
   public static final int DELTA_X = 2;
   public static final int DELTA_Y = DELTA_X;
   public static final int SPRITE_WIDTH = 10;
   public static final int SPRITE_HEIGHT = SPRITE_WIDTH;
   private static final String PRESSED = "pressed";
   private static final String RELEASED = "released";
   private static final int PREF_W = 800;
   private static final int PREF_H = 650;
   private Map<Dir, Boolean> dirMap = new EnumMap<>(Dir.class);
   private int spriteX = 0;
   private int spriteY = 0;
   private BufferedImage sprite;
   private Timer animationTimer = new Timer(TIMER_DELAY, new AnimationListener());

   private int leftWheelSpeed = 0;
   private int rightWheelSpeed = 0;
   private static final int MAXSPEED = 1000;
   private static final int MINSPEED = -1000;
   private static final int INCREMENT = 10;

   public ControlPad() {
      for (Dir dir : Dir.values()) {
         dirMap.put(dir, Boolean.FALSE);
      }

      sprite = createSprite();
      setKeyBindings();
      animationTimer.start();
  
   }

   private BufferedImage createSprite() {
      BufferedImage sprt = new BufferedImage(SPRITE_WIDTH, SPRITE_HEIGHT, BufferedImage.TYPE_INT_ARGB);
      Graphics g = sprt.getGraphics();
      g.setColor(Color.RED);
      g.fillRect(0, 0, SPRITE_WIDTH, SPRITE_HEIGHT);
      g.dispose();
      return sprt;
   }

   @Override
   public Dimension getPreferredSize() {
      return new Dimension(PREF_W, PREF_H);
   }

   @Override
   protected void paintComponent(Graphics g) {
      super.paintComponent(g);
      if (sprite != null) {
         g.drawImage(sprite, spriteX, spriteY, this);
      }
   }

   private void setKeyBindings() {
      int condition = WHEN_IN_FOCUSED_WINDOW;
      InputMap inputMap = getInputMap(condition);
      ActionMap actionMap = getActionMap();

      for (Dir dir : Dir.values()) {
         KeyStroke keyPressed = KeyStroke.getKeyStroke(dir.getKeyCode(), 0, false);
         KeyStroke keyReleased = KeyStroke.getKeyStroke(dir.getKeyCode(), 0, true);

         inputMap.put(keyPressed, dir.toString() + PRESSED);
         inputMap.put(keyReleased, dir.toString() + RELEASED);

         actionMap.put(dir.toString() + PRESSED, new DirAction(dir, PRESSED));
         actionMap.put(dir.toString() + RELEASED, new DirAction(dir, RELEASED));
      }

   }

   private class AnimationListener implements ActionListener {
	   
      @Override
      public void actionPerformed(ActionEvent e) {
    	 boolean done = false;
         int newX = spriteX;
         int newY = spriteY;
         int keyval = 0;
         for (Dir dir : Dir.values()) {
            if (dirMap.get(dir)) {
              if( dir.getKeyCode() == KeyEvent.VK_K ) {
            		leftWheelSpeed = 0;
            		rightWheelSpeed = 0;
            		keyval = 1;
              } else {
            	  if(dir.getKeyCode() == KeyEvent.VK_U) {
            		  keyval = 2;
            	  } else {
            		 if( dir.getKeyCode() == KeyEvent.VK_D) {
            			 keyval = 3;
            		 } else {
            	      keyval = 1;
                      newX += dir.getDeltaX() * DELTA_X;
                      newY += dir.getDeltaY() * DELTA_Y;
                      if( !done) {
                   	   done = true;
                   	   if( dir.getDeltaX() < 0 ) {
                   		   if( leftWheelSpeed > MINSPEED ) {
                   			   leftWheelSpeed -= INCREMENT;
                   		   }
                      } else {
                   	   	 if( dir.getDeltaX() != 0 && rightWheelSpeed < MAXSPEED) 
                   	   		 rightWheelSpeed += INCREMENT;
                      }
                      if( dir.getDeltaY() > 0 ) {
                   	   	if( leftWheelSpeed < MAXSPEED ){
                   		   leftWheelSpeed += INCREMENT;
                   	   	}
                      } else {
                   	   if( dir.getDeltaY() != 0 && rightWheelSpeed > MINSPEED) 
                   		   rightWheelSpeed -= INCREMENT;
                      }
                    } // not done
                   }  // keycode d		  
            	  } // keycode u
              } // keycode k
              switch(keyval) {
              case 0:
            	  break;
              case 1:
          		try {
          		 FileWriter fos = new FileWriter("/home/jg/coords");
          		 fos.write(String.valueOf(leftWheelSpeed)+","+String.valueOf(rightWheelSpeed));
          		 fos.flush();fos.close();
          		} catch (IOException e1) {
          		 // TODO Auto-generated catch block
          		 e1.printStackTrace();
          		}
          		break;
              case 2:
           		try {
             		 FileWriter fos = new FileWriter("/home/jg/coords");
             		 fos.write(String.valueOf(leftWheelSpeed)+","+String.valueOf(rightWheelSpeed));
             		 fos.flush();fos.close();
             	} catch (IOException e1) {
             		 // TODO Auto-generated catch block
             		 e1.printStackTrace();
             	}
             	break;
              } // switch
            } // if dirmap
         } // for
         if (newX < 0 ) {
            newX = getWidth() - SPRITE_WIDTH;
         }
         if( newY < 0) {
        	 newY = getHeight()-SPRITE_WIDTH;
         }
         if (newX + SPRITE_WIDTH > getWidth()) {
        	 newX = 0;
         }
         if(newY + SPRITE_HEIGHT > getHeight()) {
        	 newY = 0;
         }
         spriteX = newX;
         spriteY = newY;
         repaint();
  
         //twistPubs.pubdata.addLast(new int[]{leftWheelSpeed, rightWheelSpeed});
         
      }
   }

   private class DirAction extends AbstractAction {

      private String pressedOrReleased;
      private Dir dir;

      public DirAction(Dir dir, String pressedOrReleased) {
         this.dir = dir;
         this.pressedOrReleased = pressedOrReleased;
      }

      @Override
      public void actionPerformed(ActionEvent evt) {
         if (pressedOrReleased.equals(PRESSED)) {
            dirMap.put(dir, Boolean.TRUE);
         } else if (pressedOrReleased.equals(RELEASED)) {
            dirMap.put(dir, Boolean.FALSE);
         }
      }

   }

   private static void createAndShowGui() {
      ControlPad mainPanel = new ControlPad();

      JFrame frame = new JFrame("KeyBindingEg");
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      frame.getContentPane().add(mainPanel);
      frame.pack();
      frame.setLocationByPlatform(true);
      frame.setVisible(true);
  
   }

   public static void main(String[] args) {
	     //InetSocketAddress inet = new InetSocketAddress("192.168.1.4",8090);
	     // twistPubs = new TwistPubs("192.168.1.3", inet);
	     SwingUtilities.invokeLater(new Runnable() {
         public void run() {
            createAndShowGui();
         }
      });
 
   }
}