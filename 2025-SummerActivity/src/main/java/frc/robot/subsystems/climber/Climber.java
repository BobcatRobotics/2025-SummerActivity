// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final ThriftyNova climber = new ThriftyNova(Constants.Climber.DeviceID, Constants.Climber.Type);

  public Climber() {
    climber.setMaxCurrent(CurrentType.STATOR, Constants.Climber.CurrentLimit);
    climber.setMaxCurrent(CurrentType.SUPPLY, Constants.Climber.CurrentLimit);
    climber.setInverted(Constants.Climber.Inverted);
    climber.setBrakeMode(Constants.Climber.BrakeMode);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void hook() {
    climber.set(Constants.Climber.Hook);
  }

  public void release() {
    climber.set(Constants.Climber.Release);
  }

  public void stop() {
    climber.stopMotor();
  }
}
