package frc.robot.subsystems.drive.AlgaeRemover;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeRemoverSubsystem extends SubsystemBase {
  private AlgaeRemoverIO io;
  private AlgaeRemoverIOInputsAutoLogged inputs = new AlgaeRemoverIOInputsAutoLogged();
  private final Alert disconnectedAlert = new Alert("Motor Disconnected", AlertType.kWarning);
  private final String name;
  public AlgaeRemoverSubsystem(AlgaeRemoverIO io, String name) {
    this.io = io;
    this.name = name;
  }

@Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    disconnectedAlert.set(!inputs.connected);
  }

  public void runArm(double positionInRotations) {
    io.runArm(positionInRotations);
  }
  public void runRoller(double positionInRotations) {
    io.runRoller(positionInRotations);
  }

  public void stopArm() {
    io.stopArm();
  }
  public void stopRoller(){
    io.stopRoller();
  }
}
