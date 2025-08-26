// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs{
    public double armPosition = 0.0;
    
    public double armVelocity = 0.0;

    public double armAppliedVolts = 0.0;

    public double armCurrentAmps = 0.0;
  }
  public default void updateInputs(ClimberIOInputs inputs) {
    
  }

  public default void moveClimber(double speed) {

  }

  public default void stop() {

  }
}

