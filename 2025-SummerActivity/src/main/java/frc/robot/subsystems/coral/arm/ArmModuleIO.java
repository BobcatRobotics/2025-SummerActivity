// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

public interface ArmModuleIO {

  public static class ArmModuleIOInputs {
    public boolean connected = false;

    public double position = 0.0;

    public double velocity = 0.0;

    public double volts = 0.0;

    public double amps = 0.0;

    public ArmState state = ArmState.STALL;
  }

  public default void updateInputs(ArmModuleIOInputs inputs) {}

  public default void runArm(double positionInRotations) {}

  public default void stopArm() {}
}
