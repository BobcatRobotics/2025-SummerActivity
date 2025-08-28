// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.thethriftybot.ThriftyNova;
import com.thethriftybot.ThriftyNova.CurrentType;

import frc.robot.Constants;

public final class ClimberThriftyNova{
  final static ThriftyNova climberThriftyNova = new ThriftyNova(Constants.Climber.DeviceID, Constants.Climber.Type);  
      public static final class Motor{
        public static ThriftyNova MotorName = climberThriftyNova;
    }
  public ClimberThriftyNova(){
    climberThriftyNova.setMaxCurrent(CurrentType.STATOR, Constants.Climber.CurrentLimit);
    climberThriftyNova.setMaxCurrent(CurrentType.SUPPLY, Constants.Climber.CurrentLimit);
    climberThriftyNova.setInverted(Constants.Climber.Inverted);
    climberThriftyNova.setBrakeMode(Constants.Climber.BrakeMode);
  }
}