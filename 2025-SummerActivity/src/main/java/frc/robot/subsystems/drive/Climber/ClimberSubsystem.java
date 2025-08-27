package frc.robot.subsystems.drive.Climber;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  private ClimberModuleIO io;
  private ClimberModuleIOInputsAutoLogged inputs = new ClimberModuleIOInputsAutoLogged();
  private final Alert motorDisconnectedAlert = new Alert("disconnected", AlertType.kWarning);
  private final String name;

  public ClimberSubsystem(ClimberModuleIO io, String name) {
    this.io = io;
    this.name = name;
  }

  @Override
  public void periodic() {
    io.periodic();
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    motorDisconnectedAlert.set(!inputs.connected);
  }

  public void runClimber(double positionInRotations) {
    io.runClimber(positionInRotations);
  }

  public void stopClimber() {
    io.stopClimber();
  }
}
