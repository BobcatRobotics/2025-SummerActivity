// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.roller;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public final class RollerTalonFX{
  final static TalonFX rollerTalonFX = new TalonFX(Constants.Roller.DeviceID, Constants.Roller.Canbus);
  public static final class Motor{
    public static TalonFX MotorName = rollerTalonFX;
  }
  public RollerTalonFX(){
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
    rollerTalonFX.getConfigurator().apply(motor_config);
  }
}