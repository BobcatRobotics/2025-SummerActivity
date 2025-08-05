// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

public class Roller extends SubsystemBase {
  /** Creates a new Roller. */
  private final TalonFX roller = new TalonFX(0, "rio");
  private final VelocityDutyCycle velocity_request = new VelocityDutyCycle(0);
  

  public Roller() {
    //Motor Configuration
    TalonFXConfiguration motor_config = new TalonFXConfiguration();
    motor_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor_config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motor_config.CurrentLimits.StatorCurrentLimit = 57;
    motor_config.CurrentLimits.StatorCurrentLimitEnable = true;
    motor_config.CurrentLimits.SupplyCurrentLimit = 57;
    motor_config.CurrentLimits.SupplyCurrentLimitEnable = true;

    //Slot Configuration
    var Slot0Configs = new Slot0Configs();
    Slot0Configs.kP = 0.1;

    //Add Slot Configuration to Motor Configuration
    motor_config.Slot0 = Slot0Configs;

    //Apply Configuration to TalonFX roller
    roller.getConfigurator().apply(motor_config);

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
  public void stop_roller(){
    roller.stopMotor();
  }

}
