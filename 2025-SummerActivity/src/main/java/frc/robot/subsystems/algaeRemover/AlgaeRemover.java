package frc.robot.subsystems.algaeRemover;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class AlgaeRemover extends SubsystemBase {
  // final TalonFX algaeRemoverPositionMotor = new TalonFX(0, "rio");
  // final TalonFX algaeRemoverPositionMotor = new TalonFX(0, "rio");
  private final AlgaeRemoverIO io;
  private final AlgaeRemoverIOInputsAutoLogged inputs = new AlgaeRemoverIOInputsAutoLogged();
  /** Creates a new Algae Remover. */
  public AlgaeRemover() {
    if (Constants.currentMode == Mode.REAL) {
      this.io = new AlgaeRemoverIOTalonFX();
    } else {
      this.io = new AlgaeRemoverIOSimulation();
    }
  }

  public void setPosition(double position) {
    io.setPosition(position);
  }

  public void setSpeed(double speed) {
    io.setSpeed(speed);
  }

  public void stopPositionMotor() {
    io.stopPositionMotor();
  }

  public void stopWheelMotor() {
    io.stopWheelMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("algaeRemover", inputs);
  }
}
