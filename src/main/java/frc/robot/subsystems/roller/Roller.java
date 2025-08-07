// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.roller;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Roller extends SubsystemBase {
  private RollerInterfaceIO io;
  //private RollerModuleIOInputsAutoLogged inputs = new RollerModuleIOInputsAutoLogged();
  private final Alert motorDisconnectedAlert = new Alert("motor disconnected!", AlertType.kWarning);
  private final String name;
  /** Creates a new roller. */
  public Roller(RollerInterfaceIO io, String name) {
    this.io = io;
    this.name = name;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    motorDisconnectedAlert.set(!inputs.connected);
  }

  public void runRoller(double speedInRadians) {
    io.runRoller(speedInRadians);
  }

  public void stopRoller() {
    io.stopRoller();
  }
}