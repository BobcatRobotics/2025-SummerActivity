package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;

public class RollerSubsystem extends SubsystemBase {
  private final RollerIOTalonFX io;
  private final RollerIOinputsAutoLogged inputs = new RollerIOinputsAutoLogged();

  public RollerSubsystem() {
    io = new RollerIOTalonFX(RollerConstants.ROLLER_MOTOR_ID);
    // Set up the roller motor as a brushed motor
    // String Busname = ""; you dont need this, it defaults to ""

  }

  @Override
  public void periodic() {}

  public void runRoller(double speed) {
    io.runRoller(speed);
  }

  public void stopRoller() {
    io.stopRoller();
  }
}
