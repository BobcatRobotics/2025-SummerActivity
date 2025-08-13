package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RollerConstants;
import org.littletonrobotics.junction.Logger;

public class RollerSubsystem extends SubsystemBase {
  private final RollerIO io;
  private final RollerIOinputsAutoLogged inputs = new RollerIOinputsAutoLogged();

  public RollerSubsystem() {
    if (Constants.currentMode == Mode.REAL) {
      io = new RollerIOTalonFX(RollerConstants.ROLLER_MOTOR_ID);
    } else {
      io = new RollerIOSim();
    }
    // Set up the roller motor as a brushed motor
    // String Busname = ""; you dont need this, it defaults to ""

  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Roller", inputs);
  }

  public void runRoller(double speed) {
    io.runRoller(speed);
  }

  public void stopRoller() {
    io.stopRoller();
  }
}
