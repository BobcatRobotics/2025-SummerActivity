// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;
import com.thethriftybot.ThriftyNova.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final ThriftyNova climber = new ThriftyNova(1, MotorType.NEO);

  public Climber() {
    climber.setMaxCurrent(CurrentType.STATOR, 50);
    climber.setMaxCurrent(CurrentType.SUPPLY, 50);
    climber.setInverted(true);
    climber.setBrakeMode(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void hook() {
    climber.set(0.6);
  }

  public void release() {
    climber.set(-0.6);
  }

  public void stop() {
    climber.stopMotor();
  }
}
