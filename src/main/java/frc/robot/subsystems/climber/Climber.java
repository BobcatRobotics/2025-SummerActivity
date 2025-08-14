package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
  private ClimberIO io;
  private ClimberModuleIOInputsAutoLogged inputs = new ClimberModuleIOInputsAutoLogged();
  private final Alert motorDisconnectedAlert = new Alert("motor disconnected!", AlertType.kWarning);
  private final String name;

  public Climber(ClimberIO io, String name) {
    this.io = io;
    this.name = name;
  }

  @Override
  public void periodic() {
    io.periodic();
    // This method will be called once per scheduler run
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