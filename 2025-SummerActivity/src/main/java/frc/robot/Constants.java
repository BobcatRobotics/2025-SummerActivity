// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.thethriftybot.ThriftyNova.MotorType;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final class Arm{
    public static int DeviceID = 9;
    public static String Canbus = "rio";
    public static InvertedValue MotorInverted = InvertedValue.Clockwise_Positive;
    public static NeutralModeValue NeutralMode = NeutralModeValue.Coast;
    public static int CurrentLimit = 57;
    public static boolean CurrentLimitEnable = true;
    public static double RotationsPerSecondCW = 0.177;
    public static double RotationsPerSecondCCW = -0.19;
  }
  public static final class Roller{
    public static int DeviceID = 10;
    public static String Canbus = "rio";
    public static int Velocity = 0;
    public static InvertedValue MotorInverted = InvertedValue.Clockwise_Positive;
    public static NeutralModeValue NeutralMode = NeutralModeValue.Coast;
    public static int CurrentLimit = 57;
    public static boolean CurrentLimitEnable = true;
    public static double RotationsPerSecondCW = 60;
    public static double RotationsPerSecondCCW = -60;
  }
  public static final class Climber{
    public static int DeviceID = 10;
    public static MotorType Type = MotorType.NEO;
    public static int Velocity = 0;
    public static boolean Inverted = true;
    public static boolean BrakeMode = true;
    public static int CurrentLimit = 50;
    public static double Hook = 0.6;
    public static double Release = -0.6;
  }
  public static final class Dealgaefier{
    public static int Roller_DeviceID = 12;
    public static int Arm_DeviceID = 11;
    public static String Canbus = "rio";
    public static int Velocity = 0;
    public static int Roller_MechanicalRatio = 4;
    public static int Arm_MechanicalRatio = 25;
    public static InvertedValue MotorInverted = InvertedValue.Clockwise_Positive;
    public static NeutralModeValue NeutralMode = NeutralModeValue.Coast;
    public static int CurrentLimit = 57;
    public static boolean CurrentLimitEnable = true;
    public static double Roller_RotationsPerSecondCW = 60;
    public static double Roller_RotationsPerSecondCCW = -60;
    public static double Arm_RotationsPerSecondCW = 0.36;
    public static double Arm_RotationsPerSecondCCW = -0.36;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
