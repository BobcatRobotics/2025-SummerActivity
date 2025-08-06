// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
<<<<<<< Updated upstream
    /**
     * Example of an inner class. One can "import static
     * [...].Constants.OIConstants.*" to gain access
     * to the constants contained within without having to preface the names with
     * the class, greatly
     * reducing the amount of text required.
     */
    public static final class OIConstants {
        // Example: the port of the driver's controller
        public static final int DriverControllerPort = 0;
    }
=======
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class RollerConstants {
    public static final int ROLLER_MOTOR_ID = 10;
    public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
    public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
    public static final double ROLLER_SLOW_SPEED_OUT = 0.1;
    public static final double ROLLER_FAST_SPEED_OUT = 0.2;
    public static final double ROLLER_SPEED_IN = -0.1;
  }
>>>>>>> Stashed changes
}
