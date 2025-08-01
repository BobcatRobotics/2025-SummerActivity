// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Arm has to move in one direction when a button is pressed and the other direction when another
// button is pressed

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  final TalonFX armMotor = new TalonFX(0, "rio");

  /** Creates a new Arm. */
  public Arm() {
    var talonFXConfigurator = armMotor.getConfigurator();
    var limitConfigs = new CurrentLimitsConfigs();
    var motorConfigs = new MotorOutputConfigs();

    // enable stator current limit
    limitConfigs.StatorCurrentLimit = 20;
    limitConfigs.StatorCurrentLimitEnable = true;

    // set invert mode to clockwise
    motorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    motorConfigs.NeutralMode = NeutralModeValue.Coast;

    talonFXConfigurator.apply(limitConfigs);
    talonFXConfigurator.apply(motorConfigs);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kG = 0.1; // To overcome gravity
    slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative

    armMotor.getConfigurator().apply(slot0Configs);
  }

  public void armIn() {
    final PositionVoltage armMotorRequest = new PositionVoltage(0).withSlot(0);
    armMotor.setControl(armMotorRequest.withPosition(0.3));
  }

  public void armOut() {
    final PositionVoltage armMotorRequest = new PositionVoltage(0).withSlot(0);
    armMotor.setControl(armMotorRequest.withPosition(-0.3));
  }

  public void stopArm() {
    armMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
