// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private ArmInterfaceIO io;
  private ArmModuleIOInputsAutoLogged inputs = new ArmModuleIOInputsAutoLogged();
  private final Alert motorDisconnectedAlert = new Alert("motor disconnected!", AlertType.kWarning);
  /** Creates a new roller. */
  public Arm() {
    this.io = new ArmIOReal(Constants.ArmConstants.ARM_MOTOR_ID, "rio");
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    motorDisconnectedAlert.set(!inputs.connected);
  }

  public void runArm(double positionInRotations) {
    io.runArm(positionInRotations);
  }

  public void stopArm() {
    io.stopArm();
  }
}