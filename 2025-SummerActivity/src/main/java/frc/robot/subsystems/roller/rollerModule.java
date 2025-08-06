package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public class rollerModule {
  private rollerModuleIO io;
  private rollerModuleIOInputsAutoLogged inputs = new rollerModuleIOInputsAutoLogged();
  private final Alert motorDisconnectedAlert = new Alert("motor disconnected!", AlertType.kWarning);
  private final String name;
  /**
   * Constructs a new MotorBase instance.
   *
   * @param io The {@link MotorIO} implementation for controlling hardware.
   * @param name The name of the motor (used for logging).
   */
  public rollerModule(rollerModuleIO io, String name) {
    this.io = io;
    this.name = name;
  }
  /**
   * Periodically updates motor inputs and logs them, while checking and displaying connection
   * status alerts.
   */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    motorDisconnectedAlert.set(!inputs.connected);
  }

  public void runRoller(double speedInRadians) {
    io.runRoller(speedInRadians);
  }

  public void stopMotor() {
    io.stopMotor();
  }
}
