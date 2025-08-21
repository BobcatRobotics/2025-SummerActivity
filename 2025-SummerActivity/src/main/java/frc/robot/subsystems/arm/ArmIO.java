package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public class ArmIOInputs {
    public double position = 0.00;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void stopArm() {}
}
