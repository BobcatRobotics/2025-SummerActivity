// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.dealgaefier;

import org.littletonrobotics.junction.AutoLog;

public interface DealgaefierIO {
  @AutoLog
  public static class DealgaefierIOInputs{
    public double dealgaefierPosition = 0.0;
    
    public double dealgaefierVelocity = 0.0;

    public double dealgaefierAppliedVolts = 0.0;

    public double dealgaefierCurrentAmps = 0.0;
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

