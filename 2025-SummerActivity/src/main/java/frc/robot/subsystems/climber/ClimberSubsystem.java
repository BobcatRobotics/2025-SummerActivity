package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {

  // private final TalonFX armMotor;
  private final ClimberIOTalonFX io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  /** This subsytem that controls the arm. */
  public ClimberSubsystem() {
    io = new ClimberIOTalonFX();
    // Set up the arm motor as a brushed motor

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  /**
   * This is a method that makes the climber move at your desired speed Positive values make it pull
   * in to the robot and negative values would unravel the winch.
   *
   * @param speed motor speed from -1.0 to 1, with 0 stopping it
   */
  public void runClimber(double speed) {
    io.runClimber(speed);
  }

  public void stopClimber() {
    io.stopClimber();
  }
}
