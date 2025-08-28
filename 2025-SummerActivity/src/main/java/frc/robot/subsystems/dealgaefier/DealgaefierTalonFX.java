// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.dealgaefier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;

public final class DealgaefierTalonFX{
  final static TalonFX dealgaefierTalonFX = new TalonFX(Constants.Arm.DeviceID, Constants.Arm.Canbus);
  public static final class Motor{
    public static TalonFX MotorName = dealgaefierTalonFX;
  }
  public DealgaefierTalonFX(){
    TalonFXConfiguration motor_config = new TalonFXConfiguration();
    motor_config.MotorOutput.Inverted = Constants.Dealgaefier.MotorInverted;
    motor_config.MotorOutput.NeutralMode = Constants.Dealgaefier.NeutralMode;
    motor_config.Feedback.SensorToMechanismRatio = Constants.Dealgaefier.Roller_MechanicalRatio;
    motor_config.CurrentLimits.StatorCurrentLimit = Constants.Dealgaefier.CurrentLimit;
    motor_config.CurrentLimits.StatorCurrentLimitEnable = Constants.Dealgaefier.CurrentLimitEnable;
    motor_config.CurrentLimits.SupplyCurrentLimit = Constants.Dealgaefier.CurrentLimit;
    motor_config.CurrentLimits.SupplyCurrentLimitEnable = Constants.Dealgaefier.CurrentLimitEnable;

    motor_config = new TalonFXConfiguration();
    motor_config.MotorOutput.Inverted = Constants.Dealgaefier.MotorInverted;
    motor_config.MotorOutput.NeutralMode = Constants.Dealgaefier.NeutralMode;
    motor_config.Feedback.SensorToMechanismRatio = Constants.Dealgaefier.Arm_MechanicalRatio;
    motor_config.CurrentLimits.StatorCurrentLimit = Constants.Dealgaefier.CurrentLimit;
    motor_config.CurrentLimits.StatorCurrentLimitEnable = Constants.Dealgaefier.CurrentLimitEnable;
    motor_config.CurrentLimits.SupplyCurrentLimit = Constants.Dealgaefier.CurrentLimit;
    motor_config.CurrentLimits.SupplyCurrentLimitEnable = Constants.Dealgaefier.CurrentLimitEnable;

    // Slot Configuration
    var Slot0Configs = new Slot0Configs();
    Slot0Configs.kP = 0.1;

    // Add Slot Configuration to Motor Configuration
    motor_config.Slot0 = Slot0Configs;

    // Apply Configuration to TalonFX arm
    dealgaefierTalonFX.getConfigurator().apply(motor_config);
  }
}