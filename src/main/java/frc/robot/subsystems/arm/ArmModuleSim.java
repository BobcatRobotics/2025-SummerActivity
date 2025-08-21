package frc.robot.subsystems.arm;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
//import org.littletonrobotics.junction.Logger;
//import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ArmModuleSim implements ArmModuleIO {

    private final TalonFX motor;
    private final StatusSignal<Angle> relativePos;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorAppVolts;
    private final StatusSignal<Current> motorCurr;
    private final Debouncer connectDebounce = new Debouncer(0.5);
    private final TalonFXConfiguration config;
    private final SingleJointedArmSim armSim;

    public ArmModuleSim(int id, String bus){
      this.motor = new TalonFX(9,"rio");
      config = new TalonFXConfiguration();
      config.Feedback.SensorToMechanismRatio = 75;
      config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.CurrentLimits.SupplyCurrentLimit = Constants.ArmConstants.ARM_MOTOR_CURRENT_LIMIT;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;

      motor.getConfigurator().apply(config, 0.25);

      relativePos = motor.getPosition();
      motorVelocity = motor.getVelocity();
      motorAppVolts = motor.getMotorVoltage();
      motorCurr = motor.getStatorCurrent();

      double armLength = 0.5;
      armSim = new SingleJointedArmSim(DCMotor.getKrakenX60(1),
      config.Feedback.SensorToMechanismRatio,
      SingleJointedArmSim.estimateMOI(armLength, 5),
      armLength, 
      Units.degreesToRadians(0), 
      Units.degreesToRadians(1.5707963267948966), 
      true, Units.degreesToRadians(0));
    }
    
    public void changeInputs(ArmModuleInputs input){
      armSim.setInput(motor.getMotorVoltage().getValueAsDouble());
      armSim.update(0.020);
      var motorStatus = 
        BaseStatusSignal.refreshAll(
            relativePos, motorVelocity, motorAppVolts, motorCurr);

            input.connected = connectDebounce.calculate(motorStatus.isOK());
            input.positionRadius = Units.rotationsToRadians(relativePos.getValueAsDouble());
            input.VelocityRadiusPerSec = Units.rotationsToRadians(motorVelocity.getValueAsDouble());
            input.appliedVolts = motorAppVolts.getValueAsDouble();
            input.Amps = motorCurr.getValueAsDouble();

      if (input.appliedVolts > 0) {
        input.state = ArmState.FORWARD;
      } 
      else if (input.appliedVolts < 0) {
        input.state = ArmState.REVERSE;
      } 
      else if (input.appliedVolts == 0) {
        input.state = ArmState.IDLE;
      } 
      else {
        input.state = ArmState.UNKNOWN;
      }
    }

    public void runArm(double positionInRotations){
      motor.set(positionInRotations);
      Logger.recordOutput("/Arm/positionInRotations", positionInRotations);
    }
    public void stopArm(){
        runArm(0);
        motor.stopMotor();
        Logger.recordOutput("/Arm/velocityRotPerSec", 0);
    }
    public void periodic(){
      
    }
}
