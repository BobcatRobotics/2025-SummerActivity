// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.roller;

public interface RollerModuleIO {

  public static class RollerModuleIOInputs {
    public boolean connected = false;

    public double position = 0.0;

    public double velocity = 0.0;

    public double volts = 0.0;

    public double amps = 0.0;

    public RollerState state = RollerState.STALL;
  }

  public default void updateInputs(RollerModuleIOInputs inputs) {}

  public default void runRoller(double positionInRotations) {}

  public default void stopRoller() {}
}
