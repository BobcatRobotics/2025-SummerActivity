// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public Climber() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void hook() {
    ClimberThriftyNova.climberThriftyNova.set(Constants.Climber.Hook);
  }

  public void release() {
    ClimberThriftyNova.climberThriftyNova.set(Constants.Climber.Release);
  }

  public void stop() {
    ClimberThriftyNova.climberThriftyNova.stopMotor();
  }
}
