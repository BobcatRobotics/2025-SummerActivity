package frc.robot.subsystems.algaeRemover;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeRemover extends SubsystemBase {
  private AlgaeRemoverIO io;
  private AlgaeRemoverIOInputsAutoLogged inputs = new AlgaeRemoverIOInputsAutoLogged();
  private final Alert motorDisconnectedAlert = new Alert("motor disconnected!", AlertType.kWarning);
  private final String name;
  /** Creates a new roller. */
  public AlgaeRemover(AlgaeRemoverIO io, String name) {
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