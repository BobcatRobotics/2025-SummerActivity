package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
// import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.roller.RollerIOSim;

public class ArmSubsystem extends SubsystemBase {

  private final ArmIO io;
  private final ArmIOinputsAutoLogged inputs = new ArmIOinputsAutoLogged();

  public ArmSubsystem() {
    if (Constants.currentMode == Mode.REAL) {
      io = new ArmIOTalonFX(RollerConstants.ROLLER_MOTOR_ID);
    } else {
      io = new ArmIOTalonFX(RollerConstants.ROLLER_MOTOR_ID); //FIX THIS ONCE SIM CLASS IS CREATED
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Roller", inputs);
  }

  public void positionArm(double pos) {
    io.positionArm(pos);
  }

  public void stopArm() {
    io.stopArm();
  }
}
