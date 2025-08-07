// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Subsystem needs to

package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Roller extends SubsystemBase {
  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();
  /** Creates a new Roller. */
  public Roller() {
    if (Constants.currentMode == Mode.REAL) {
      this.io = new RollerIOTalonFX();
    } else {
      this.io = new RollerIOSimulation();
    }
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  public void stopRoller() {
    io.stopRoller();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Roller", inputs);
  }
}
