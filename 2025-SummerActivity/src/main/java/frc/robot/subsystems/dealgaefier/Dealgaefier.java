// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.dealgaefier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Dealgaefier extends SubsystemBase {
  /** Creates a new Dealgaefier. */
  public Dealgaefier() {}

  public void start_roller_clockwise() {
    spin_arm(Constants.Dealgaefier.Roller_RotationsPerSecondCW);
  }

  public void start_roller_counterclockwise() {
    spin_arm(Constants.Dealgaefier.Roller_RotationsPerSecondCCW);
  }

  public void start_arm_clockwise() {
    spin_arm(Constants.Dealgaefier.Arm_RotationsPerSecondCW);
  }

  public void start_arm_counterclockwise() {
    spin_arm(Constants.Dealgaefier.Arm_RotationsPerSecondCCW);
  }
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Sets speed of motor
  public void spin_arm(double rotations_per_second) {
    DealgaefierTalonFX.dealgaefierTalonFX.set(rotations_per_second);
  }
  // Stops motor
  public void stop_arm() {
    DealgaefierTalonFX.dealgaefierTalonFX.stopMotor();
  //Sets speed of motor
  }
  public void spin_roller(double rotations_per_second) {
    DealgaefierTalonFX.dealgaefierTalonFX.set(rotations_per_second);
  }
  // Stops motor
  public void stop_roller() {
    DealgaefierTalonFX.dealgaefierTalonFX.stopMotor();
  }
}
