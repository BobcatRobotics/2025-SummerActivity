// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.dealgaefier;

import org.littletonrobotics.junction.AutoLog;

public interface DealgaefierIO {
  @AutoLog
  public static class DealgaefierIOInputs{
    public double armPosition = 0.0;
    
    public double armVelocity = 0.0;

    public double armAppliedVolts = 0.0;

    public double armCurrentAmps = 0.0;
  }
  public default void updateInputs(DealgaefierIOInputs inputs) {
    
  }

  public default void spinArm(double speed) {

  }

  public default void stopArm() {

  }

  public default void spinRoller(){

  }

  public default void stopRoller(){

  }
}

