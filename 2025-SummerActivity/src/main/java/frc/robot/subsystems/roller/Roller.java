// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.roller;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Roller extends SubsystemBase {
  /** Creates a new Roller. */
  private final TalonFX roller = new TalonFX(10, "rio");

  private final VelocityDutyCycle velocity_request = new VelocityDutyCycle(Constants.Roller.Velocity);

  public Roller() {
    // Motor Configuration
    TalonFXConfiguration motor_config = new TalonFXConfiguration();
    motor_config.MotorOutput.Inverted = Constants.Roller.MotorInverted;
    motor_config.MotorOutput.NeutralMode = Constants.Roller.NeutralMode;
    motor_config.CurrentLimits.StatorCurrentLimit = Constants.Roller.CurrentLimit;
    motor_config.CurrentLimits.StatorCurrentLimitEnable = Constants.Roller.CurrentLimitEnable;
    motor_config.CurrentLimits.SupplyCurrentLimit = Constants.Roller.CurrentLimit;
    motor_config.CurrentLimits.SupplyCurrentLimitEnable = Constants.Roller.CurrentLimitEnable;

    // Slot Configuration
    var Slot0Configs = new Slot0Configs();
    Slot0Configs.kP = 0.1;

    // Add Slot Configuration to Motor Configuration
    motor_config.Slot0 = Slot0Configs;

    // Apply Configuration to TalonFX roller
    roller.getConfigurator().apply(motor_config);
  }

  public void start_clockwise() {
    spin_roller(Constants.Roller.RotationsPerSecondCW);
  }

  public void start_counterclockwise() {
    spin_roller(Constants.Roller.RotationsPerSecondCCW);
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

  // Stops motor
  public void stop_roller() {
    roller.stopMotor();
  }
}
