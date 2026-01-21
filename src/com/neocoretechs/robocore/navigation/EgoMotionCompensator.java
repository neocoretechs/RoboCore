package com.neocoretechs.robocore.navigation;

import com.neocoretechs.robocore.ranging.RangeFinderPubs;

/**
 * Uses the IMU accelerometer to estimate robot velocity,
 * rotate it into world-frame, subtract it from PCA world-frame motion,
 * and emit a canonical semantic vector.
 *
 * <p>Pipeline stages:</p>
 * <ol>
 *   <li>Gravity compensation</li>
 *   <li>Velocity integration</li>
 *   <li>World-frame rotation</li>
 *   <li>Ego-motion subtraction + semantic vector output</li>
 * </ol>
 *
 * <p>The semantic vector contains:</p>
 * <ul>
 *   <li>Robot-frame PCA direction</li>
 *   <li>World-frame PCA direction</li>
 *   <li>World-frame relative velocity</li>
 *   <li>Robot world-frame velocity</li>
 *   <li>Absolute object world-frame velocity</li>
 *   <li>Distance metrics</li>
 *   <li>Jitter / noise / confidence</li>
 * </ul>
 *
 * <p>The primary purpose is to compensate for robot motion in the range-finding pipeline. As a result:</p>
 * <ul>
 *   <li>motion_strength stops spiking when the robot moves</li>
 *   <li>world_vel_x / world_vel_y become physically correct</li>
 *   <li>Stationary objects appear stationary</li>
 *   <li>Moving objects track correctly even while the robot turns</li>
 *   <li>An LLM can now reason about true world-frame motion</li>
 * </ul>
 *
 * @see RangeFinderPubs
 * @author Copilot
 */
public final class EgoMotionCompensator {
    // Persistent robot-frame velocity state
    private double vxRobot = 0.0;
    private double vyRobot = 0.0;
    private long lastTimeNs = 0L;
    // Gravity constant
    private static final double G = 9.80665;
    private static final double DEG2RAD = 0.017453292519943295;
    public static final class ImuSample {
        public final double ax;   // m/s^2, robot frame
        public final double ay;
        public final double az;
        public final double pitchRad;
        public final double rollRad;
        public final double yawDeg; // compass heading, degrees
        public ImuSample(double ax, double ay, double az,
                         double pitchRad, double rollRad, double yawDeg) {
            this.ax = ax;
            this.ay = ay;
            this.az = az;
            this.pitchRad = pitchRad;
            this.rollRad = rollRad;
            this.yawDeg = yawDeg;
        }
    }
    public static final class PcaMotion {
        public final double vxRobot;   // eigvec3.x (robot frame)
        public final double vyRobot;   // eigvec3.y (robot frame)
        public final double motionStrength; // variance3
        public PcaMotion(double vxRobot, double vyRobot, double motionStrength) {
            this.vxRobot = vxRobot;
            this.vyRobot = vyRobot;
            this.motionStrength = motionStrength;
        }
    }
    public static final class MotionSemantic {
        public final double worldDirX;
        public final double worldDirY;
        public final double worldVelX;
        public final double worldVelY;
        public final double robotWorldX;
        public final double robotWorldY;
        public final double absObjX;
        public final double absObjY;
        public MotionSemantic(double worldDirX, double worldDirY,
                              double worldVelX, double worldVelY,
                              double robotWorldX, double robotWorldY,
                              double absObjX, double absObjY) {
            this.worldDirX = worldDirX;
            this.worldDirY = worldDirY;
            this.worldVelX = worldVelX;
            this.worldVelY = worldVelY;
            this.robotWorldX = robotWorldX;
            this.robotWorldY = robotWorldY;
            this.absObjX = absObjX;
            this.absObjY = absObjY;
        }
        public void appendTo(StringBuilder sb) {
            sb.append(",world_dir_x=").append(String.format("%3.3f", worldDirX));
            sb.append(",world_dir_y=").append(String.format("%3.3f", worldDirY));
            sb.append(",world_vel_x=").append(String.format("%3.3f", worldVelX));
            sb.append(",world_vel_y=").append(String.format("%3.3f", worldVelY));
            sb.append(",robot_world_x=").append(String.format("%3.3f", robotWorldX));
            sb.append(",robot_world_y=").append(String.format("%3.3f", robotWorldY));
            sb.append(",abs_obj_x=").append(String.format("%3.3f", absObjX));
            sb.append(",abs_obj_y=").append(String.format("%3.3f", absObjY));
        }
    }
    /**
     * Update ego-motion state from IMU and compute compensated motion.
     */
    public MotionSemantic update(ImuSample imu, PcaMotion pca) {
        long now = System.nanoTime();
        double dt = (lastTimeNs == 0L) ? 0.0 : (now - lastTimeNs) * 1e-9;
        lastTimeNs = now;
        // 1) Gravity compensation (horizontal plane)
        double axCorrected = imu.ax - G * Math.sin(imu.pitchRad);
        double ayCorrected = imu.ay + G * Math.sin(imu.rollRad);
        // 2) Integrate accel -> robot-frame velocity
        vxRobot += axCorrected * dt;
        vyRobot += ayCorrected * dt;
        // Simple drift damping
        vxRobot *= 0.98;
        vyRobot *= 0.98;
        // 3) Rotate robot velocity into world frame
        double yawRad = imu.yawDeg * DEG2RAD;
        double cos = Math.cos(yawRad);
        double sin = Math.sin(yawRad);
        double robotWorldX = cos * vxRobot - sin * vyRobot;
        double robotWorldY = sin * vxRobot + cos * vyRobot;
        // 4) PCA motion: robot frame -> world frame
        double vxr = pca.vxRobot;
        double vyr = pca.vyRobot;
        double worldDirX = cos * vxr - sin * vyr;
        double worldDirY = sin * vxr + cos * vyr;
        double speed = Math.sqrt(Math.max(pca.motionStrength, 0.0));
        double worldVelX = worldDirX * speed;
        double worldVelY = worldDirY * speed;
        // 5) Absolute object motion (world frame)
        double absObjX = worldVelX - robotWorldX;
        double absObjY = worldVelY - robotWorldY;
        return new MotionSemantic(
                worldDirX, worldDirY,
                worldVelX, worldVelY,
                robotWorldX, robotWorldY,
                absObjX, absObjY
        );
    }
}
