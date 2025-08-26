package frc.robot.subsystems.dealgifier;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO{
  @AutoLog
  public static class AlgaeInputs {
    
    public boolean connected = false; 
    public double positionRadius = 0.0;
    public double VelocityRadiusPerSec = 0.0;
    public double appliedVolts = 0.0; //volts applied
    public double Amps = 0.0;
    public AlgaeStates state = AlgaeStates.IDLE; //State of motor

    public boolean Rollerconnected = false; 
    public double RollerpositionRadius = 0.0;
    public double RollerVelocityRadiusPerSec = 0.0;
    public double RollerappliedVolts = 0.0; //volts applied
    public double RollerAmps = 0.0;
    public AlgaeStates Rollerstate = AlgaeStates.IDLE; //State of motor

    public AlgaeStates rollerState = AlgaeStates.IDLE;
    public AlgaeStates armState = AlgaeStates.IDLE;
  }


}

