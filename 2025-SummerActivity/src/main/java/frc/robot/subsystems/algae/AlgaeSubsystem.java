package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AlgaeSubsystem extends SubsystemBase {

  private final AlgaeIO io;
  private final AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();

  public AlgaeSubsystem() {
    io = new AlgaeIOTalonFX();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Algae", inputs);
  }

  public void runArm(double speed) {
    io.runArm(speed);
  }

  public void stopArm() {
    io.stopArm();
  }

  public void runRoller(double speed) {
    io.runRoller(speed);
  }

  public void stopRoller() {
    io.stopRoller();
  }
}
