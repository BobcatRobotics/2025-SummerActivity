package frc.robot.subsystems.dealgaefier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class DeAlgaefierSubsystem extends SubsystemBase {

  private final DeAlgaefierIO io;
  private final DeAlgaefierIOinputsAutoLogged inputs = new DeAlgaefierIOinputsAutoLogged();

  public DeAlgaefierSubsystem() {
    io = new DeAlgaefierIOTalonFX();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Roller", inputs);
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
