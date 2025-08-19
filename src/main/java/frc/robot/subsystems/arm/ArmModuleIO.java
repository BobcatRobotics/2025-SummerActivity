package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmModuleIO {

  @AutoLog
  public static class ArmModuleInputs {
    public boolean connected = false; 
    public double positionRadius = 0.0;
    public double VelocityRadiusPerSec = 0.0;
    public double appliedVolts = 0.0; //volts applied
    public double Amps = 0.0;
    public ArmState state = ArmState.IDLE; //State of motor
  }

  public default void changeInputs(ArmModuleInputs input) {}
  public default void stopArm(){}
  public default void extended(double positionInRotations){}
  public default void extended(double positionInRotations, double Velocity){}
}