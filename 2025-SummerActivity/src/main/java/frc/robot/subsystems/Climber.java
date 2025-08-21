// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final ThriftyNova climber = new ThriftyNova(1);

  public Climber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
