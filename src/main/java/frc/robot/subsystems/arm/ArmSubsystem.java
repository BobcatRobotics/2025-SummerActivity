package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
  private ArmModuleIO io;
  private ArmModuleInputsAutoLogged inputs = new ArmModuleInputsAutoLogged();
  private final Alert motorDisconnectedAlert = new Alert("motor disconnected!", AlertType.kWarning);
  private final String name;

  //private final double kExtendedPosition = 5;
  //private final double kStowedPosition = 0.0;

  public ArmSubsystem(ArmModuleIO io, String name) {
    this.io = io;
    this.name = name;
  }


  //Method to stop Motor
  public void stopMotor(){
    io.stopArm();
    
  }

  //Method to set arm position
  public void extend(double positionInRotations){
    io.extended(positionInRotations);
  }

    @Override
  public void periodic() {
    io.changeInputs(inputs);
    Logger.processInputs(name, inputs);
    motorDisconnectedAlert.set(!inputs.connected);

    // This method will be called once per scheduler run
  }
}