// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

public class RollerSubsystem extends SubsystemBase {
  private rollerModule module;

  /** Creates a new roller. */
  public RollerSubsystem() {
    if (Constants.currentMode == Mode.SIM) {

      module =
          new rollerModule(
              new rollerModuleSim(Constants.RollerConstants.ROLLER_MOTOR_ID, "rio"), "roller");
    } else {

      module =
          new rollerModule(
              new rollerModuleReal(Constants.RollerConstants.ROLLER_MOTOR_ID, "rio"), "roller");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    module.periodic();
  }

  public void runRoller(double speedInRadians) {
    module.runRoller(speedInRadians);
  }

  public void stopRoller() {
    module.stopMotor();
  }
}
