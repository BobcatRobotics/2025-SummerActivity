// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Roller extends SubsystemBase {
  /** Creates a new Roller. */
  public Roller() {}

  public void start_clockwise() {
    spin_roller(Constants.Roller.RotationsPerSecondCW);
  }

  public void start_counterclockwise() {
    spin_roller(Constants.Roller.RotationsPerSecondCCW);
  }

  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Sets speed of motor
  public void spin_roller(double rotations_per_second) {
    RollerTalonFX.rollerTalonFX.set(rotations_per_second);
  }
  // Stops motor
  public void stop_roller() {
    RollerTalonFX.rollerTalonFX.stopMotor();
  }
}
