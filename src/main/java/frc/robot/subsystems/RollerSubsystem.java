// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private final TalonFX m_MainMotor = new TalonFX(0, "rio"); //Added a single TalonFX motor
  private final VelocityDutyCycle m_MotorRequest = new VelocityDutyCycle(0);
  //private final DutyCycleOut m_MotorRequest = new DutyCycleOut(0);
  //Used for Duty Cycle
  

  public RollerSubsystem() {
    //Config setting 1: Make new configuration for motor
    TalonFXConfiguration config = new TalonFXConfiguration();
    //Config setting 1: Set inverted direction
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    //Config setting 1: Coast motion
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    //Config setting 3+4: Set Stator/Supply
    config.CurrentLimits.StatorCurrentLimit =  20;
    config.CurrentLimits.StatorCurrentLimitEnable =  true;

    m_MainMotor.getConfigurator().apply(config); //Applies motor configurations
  }

  public void spin_roller(double output) {
    m_MotorRequest.withVelocity(output);
    m_MainMotor.setControl(m_MotorRequest);
}
public void stop_motor() {
  m_MainMotor.stopMotor();
}
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

