// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final TalonFX arm = new TalonFX(Constants.Arm.DeviceID, Constants.Arm.Canbus);

  public Arm() {
    // Motor Configuration
    TalonFXConfiguration motor_config = new TalonFXConfiguration();
    motor_config.MotorOutput.Inverted = Constants.Arm.MotorInverted;
    motor_config.MotorOutput.NeutralMode = Constants.Arm.NeutralMode;
    motor_config.CurrentLimits.StatorCurrentLimit = Constants.Arm.CurrentLimit;
    motor_config.CurrentLimits.StatorCurrentLimitEnable = Constants.Arm.CurrentLimitEnable;
    motor_config.CurrentLimits.SupplyCurrentLimit = Constants.Arm.CurrentLimit;
    motor_config.CurrentLimits.SupplyCurrentLimitEnable = Constants.Arm.CurrentLimitEnable;

    // Slot Configuration
    var Slot0Configs = new Slot0Configs();
    Slot0Configs.kP = 0.1;

    // Add Slot Configuration to Motor Configuration
    motor_config.Slot0 = Slot0Configs;

    // Apply Configuration to TalonFX arm
    arm.getConfigurator().apply(motor_config);
  }

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
    arm.set(rotations_per_second);
  }
  // Stops motor
  public void stop_arm() {
    arm.stopMotor();
  }
}
