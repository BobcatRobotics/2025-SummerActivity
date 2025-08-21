// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Algae extends SubsystemBase {
  private final AlgaeIO io;
  private final AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();
  /** Creates a new Algae. */
  public Algae() {
    if (Constants.currentMode == Mode.REAL) {
      this.io = new AlgaeIOTalonFX();
    } else {
      this.io = new AlgaeIOSimulation();
    }
  }

  public void setArmSpeed(double speed) {
    io.setArmSpeed(speed);
  }

  public void setRollerSpeed(double speed) {
    io.setRollerSpeed(speed);
  }

  public void stopRoller() {
    io.stopRoller();
  }

  public void stopArm() {
    io.stopArm();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Algae", inputs);
  }
}
