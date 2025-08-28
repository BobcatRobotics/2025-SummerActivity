// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public final class ArmTalonFX{
  final static TalonFX armTalonFX = new TalonFX(Constants.Arm.DeviceID, Constants.Arm.Canbus);
  public static final class Motor{
    public static TalonFX MotorName = armTalonFX;
  }
  public ArmTalonFX(){
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
    armTalonFX.getConfigurator().apply(motor_config);
  }
}