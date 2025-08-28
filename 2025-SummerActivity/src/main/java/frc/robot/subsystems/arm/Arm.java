// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  public Arm() {}

  public void start_clockwise() {
    spin_arm(Constants.Arm.RotationsPerSecondCW);
  }

  public void start_counterclockwise() {
    spin_arm(Constants.Arm.RotationsPerSecondCCW);
  }

  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Sets speed of motor
  public void spin_arm(double rotations_per_second) {
    ArmTalonFX.armTalonFX.set(rotations_per_second);
  }
  // Stops motor
  public void stop_arm() {
    ArmTalonFX.armTalonFX.stopMotor();
  }
}
