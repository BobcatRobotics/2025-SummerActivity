// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Arm has to move in one direction when a button is pressed and the other direction when another
// button is pressed

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  final TalonFX armMotor = new TalonFX(9, "rio");
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  /** Creates a new Arm. */
  public Arm() {
    if (Constants.currentMode == Mode.REAL) {
      this.io = new ArmIOTalonFX();
    } else {
      this.io = new ArmIOSimulation();
    }
  }

  public void setPosition(double position) {
    io.setPosition(position);
  }

  public void stopArm() {
    io.stopArm();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }
}
