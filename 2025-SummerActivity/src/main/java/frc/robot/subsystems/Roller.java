// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Subsystem needs to

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Roller extends SubsystemBase {
  final TalonFX rollerMotor = new TalonFX(0, "rio");

  /** Creates a new Roller. */
  public Roller() {
    var talonFXConfigurator = rollerMotor.getConfigurator();
    var limitConfigs = new CurrentLimitsConfigs();
    var motorConfigs = new MotorOutputConfigs();

    // enable stator current limit
    limitConfigs.StatorCurrentLimit = 20;
    limitConfigs.StatorCurrentLimitEnable = true;

    // set invert mode to counter-clockwise
    motorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfigs.NeutralMode = NeutralModeValue.Coast;

    talonFXConfigurator.apply(limitConfigs);
    talonFXConfigurator.apply(motorConfigs);
  }

  public void setSpeed(double speed) {
    final DutyCycleOut rollerMotorRequest = new DutyCycleOut(speed);
    rollerMotor.setControl(rollerMotorRequest);
  }

  public void stopRoller() {
    rollerMotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
