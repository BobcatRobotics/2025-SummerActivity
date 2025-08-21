package frc.robot.subsystems.algaeRemover;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeRemoverIO {
  @AutoLog
  public class AlgaeRemoverIOInputs {
    public double position = 0.00;
    public double speed = 0.00;
  }

  public default void updateInputs(AlgaeRemoverIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void setSpeed(double speed) {}

  public default void stopPositionMotor() {}

  public default void stopWheelMotor() {}
}
