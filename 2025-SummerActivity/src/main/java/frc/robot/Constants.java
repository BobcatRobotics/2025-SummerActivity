// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
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
    public static final double ROLLER_SLOW_SPEED_OUT_IN_RADPERSEC = 25;
    public static final double ROLLER_FAST_SPEED_OUT_IN_RADPERSEC  = 100;
    public static final double ROLLER_SPEED_IN = -25;
  }
  public static final class ArmConstants {
    public static final int ARM_MOTOR_ID = 9;
    public static final int ARM_MOTOR_CURRENT_LIMIT = 10;
    public static final double ARM_MOTOR_VOLTAGE_COMP = 10;
    public static final double ARM_SPEED_DOWN = 0.177;
    public static final double ARM_SPEED_UP = -0.19;
    public static final int ARM_SWITCH_PORT = 0;
    public static final double ARM_MOTOR_STATOR_CURRENT_LIMIT = 10;
  }
  public static final class ClimberConstants {
    public static final int CLIMBER_MOTOR_ID = 1;
    public static final double CLIMBER_SUPPLY = 50;
    public static final double CLIMBER_STATOR = 50;
    public static final double CLIMBER_SPEED_OUT = -0.6;
    public static final double CLIMBER_SPEED_IN = 0.6;
  }
  public static final class AlgaeRemoverConstants{
    public static final int ARM_ID = 11;
    public static final double ARM_STATOR_LIMIT = 10;
    public static final double ARM_SPEED_UP = -0.19;
    public static final double ARM_PID_POSITION = 10;

    public static final int ROLLER_ID = 12;
    public static final double ROLLER_SPEED_OUT = 0.1;
    public static final double ROLLER_SPEED_IN = -0.1;
    public static final double ROLLER_PID_RPS = 10;
    public static final double ROLLER_MOTOR_CURRENT_LIMIT = 60;

  }
}
