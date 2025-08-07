package frc.robot.subsystems.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  public class RollerIOInputs {
    public double velocity = 0.00;
  }

  public default void updateInputs(RollerIOInputs inputs) {}

  public default void setSpeed(double speed) {}

  public default void stopRoller() {}
}
