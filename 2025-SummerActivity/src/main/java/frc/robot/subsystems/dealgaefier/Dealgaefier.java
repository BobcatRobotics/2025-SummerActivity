// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.dealgaefier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Dealgaefier extends SubsystemBase {
  /** Creates a new Roller and Arm. */
  private final TalonFX roller = new TalonFX(Constants.Dealgaefier.Roller_DeviceID, Constants.Dealgaefier.Canbus);

  private final TalonFX arm = new TalonFX(Constants.Dealgaefier.Arm_DeviceID, Constants.Dealgaefier.Canbus);

  private final VelocityDutyCycle velocity_request = new VelocityDutyCycle(Constants.Dealgaefier.Velocity);

  public Dealgaefier() {
    // Motor Configuration
    TalonFXConfiguration motor_config = new TalonFXConfiguration();
    motor_config.MotorOutput.Inverted = Constants.Dealgaefier.MotorInverted;
    motor_config.MotorOutput.NeutralMode = Constants.Dealgaefier.NeutralMode;
    motor_config.Feedback.SensorToMechanismRatio = Constants.Dealgaefier.Roller_MechanicalRatio;
    motor_config.CurrentLimits.StatorCurrentLimit = Constants.Dealgaefier.CurrentLimit;
    motor_config.CurrentLimits.StatorCurrentLimitEnable = Constants.Dealgaefier.CurrentLimitEnable;
    motor_config.CurrentLimits.SupplyCurrentLimit = Constants.Dealgaefier.CurrentLimit;
    motor_config.CurrentLimits.SupplyCurrentLimitEnable = Constants.Dealgaefier.CurrentLimitEnable;

    // Slot Configuration
    var Slot0Configs = new Slot0Configs();
    Slot0Configs.kP = 0.1;

    // Add Slot Configuration to Motor Configuration
    motor_config.Slot0 = Slot0Configs;

    // Apply Configuration to TalonFX roller
    roller.getConfigurator().apply(motor_config);

    // Motor Configuration
    motor_config = new TalonFXConfiguration();
    motor_config.MotorOutput.Inverted = Constants.Dealgaefier.MotorInverted;
    motor_config.MotorOutput.NeutralMode = Constants.Dealgaefier.NeutralMode;
    motor_config.Feedback.SensorToMechanismRatio = Constants.Dealgaefier.Arm_MechanicalRatio;
    motor_config.CurrentLimits.StatorCurrentLimit = Constants.Dealgaefier.CurrentLimit;
    motor_config.CurrentLimits.StatorCurrentLimitEnable = Constants.Dealgaefier.CurrentLimitEnable;
    motor_config.CurrentLimits.SupplyCurrentLimit = Constants.Dealgaefier.CurrentLimit;
    motor_config.CurrentLimits.SupplyCurrentLimitEnable = Constants.Dealgaefier.CurrentLimitEnable;

    // Slot Configuration
    Slot0Configs = new Slot0Configs();
    Slot0Configs.kP = 0.1;

    // Add Slot Configuration to Motor Configuration
    motor_config.Slot0 = Slot0Configs;

    arm.getConfigurator().apply(motor_config);
  }

  public void start_clockwise_roller() {
    spin_roller(Constants.Dealgaefier.Roller_RotationsPerSecondCW);
  }

  public void start_counterclockwise_roller() {
    spin_roller(Constants.Dealgaefier.Roller_RotationsPerSecondCCW);
  }

  public void start_clockwise_arm() {
    turn_arm(Constants.Dealgaefier.Arm_RotationsPerSecondCW);
  }

  public void start_counterclockwise_arm() {
    turn_arm(Constants.Dealgaefier.Arm_RotationsPerSecondCCW);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Sets speed of motor
  public void spin_roller(double rotations_per_second) {
    velocity_request.withVelocity(rotations_per_second);
    roller.setControl(velocity_request);
  }

  public void turn_arm(double rotations_per_second) {
    velocity_request.withVelocity(rotations_per_second);
    arm.setControl(velocity_request);
  }

  // Stops motor
  public void stop_roller() {
    roller.stopMotor();
  }

  public void stop_arm() {
    arm.stopMotor();
  }
}
