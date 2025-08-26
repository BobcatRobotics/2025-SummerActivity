// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Arm has to move in one direction when a button is pressed and the other direction when another
// button is pressed

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  /** Creates a new Arm. */
  public Climber() {
    if (Constants.currentMode == Mode.REAL) {
      this.io = new ClimberIOTalonFX();
    } else {
      this.io = new ClimberIOSimulation();
    }
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  public void stopClimber() {
    io.stopClimber();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }
}
