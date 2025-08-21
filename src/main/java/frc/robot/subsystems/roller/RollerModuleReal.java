package frc.robot.subsystems.roller;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
//import org.littletonrobotics.junction.Logger;
//import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.DutyCycleOut;
//import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class RollerModuleReal implements RollerModuleIO {

    private final TalonFX motor;
    private final StatusSignal<Angle> relativePos;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorAppVolts;
    private final StatusSignal<Current> motorCurr;
    private final Debouncer connectDebounce = new Debouncer(0.5);
    private final TalonFXConfiguration config;
  
    //private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    //private final PositionVoltage positionControl = new PositionVoltage(0);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);

    public RollerModuleReal(int id, String bus){
      this.motor = new TalonFX(9,"rio");
      config = new TalonFXConfiguration();
      config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.CurrentLimits.StatorCurrentLimit = Constants.RollerConstants.ROLLOR_MOTOR_CURRENT_LIMIT;
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      config.Slot0.kP = 0.5;

      motor.getConfigurator().apply(config, 0.25);

      relativePos = motor.getPosition();
      motorVelocity = motor.getVelocity();
      motorAppVolts = motor.getMotorVoltage();
      motorCurr = motor.getStatorCurrent();
    }
    
    public void changeInputs(RollerModuleInputs input){
      var motorStatus = 
        BaseStatusSignal.refreshAll(
            relativePos, motorVelocity, motorAppVolts, motorCurr);

            input.connected = connectDebounce.calculate(motorStatus.isOK());
            input.positionRadius = Units.rotationsToRadians(relativePos.getValueAsDouble());
            input.VelocityRadiusPerSec = Units.rotationsToRadians(motorVelocity.getValueAsDouble());
            input.appliedVolts = motorAppVolts.getValueAsDouble();
            input.Amps = motorCurr.getValueAsDouble();

      if (input.appliedVolts > 0) {
        input.state = RollerState.FORWARD;
      } 
      else if (input.appliedVolts < 0) {
        input.state = RollerState.REVERSE;
      } 
      else if (input.appliedVolts == 0) {
        input.state = RollerState.IDLE;
      } 
      else {
        input.state = RollerState.UNKNOWN;
      }
    }

    public void Roll(double speed){
      double velocityRotPerSec = Units.radiansToRotations(speed);
      motor.setControl(velocityControl.withVelocity(velocityRotPerSec));
      Logger.recordOutput("/Roller/velocityRotPerSec", velocityRotPerSec);
    }
    public void stopRoller(){
      motor.stopMotor();
      Logger.recordOutput("/Roller/velocityRotPerSec", 0);
    }
    public void periodic(){

    }
}
