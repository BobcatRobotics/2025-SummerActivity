package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.subsystems.arm.ArmModuleIO.ArmModuleIOInputs;

public class ArmModuleSim implements ArmModuleIO {

  private final TalonFX motor;
  private final StatusSignal<Angle> relativePosition;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;
  private final Debouncer connectedDebounce = new Debouncer(0.5);
  private final TalonFXConfiguration config;

  // Control mode objects reused for performance
  private DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private PositionVoltage positionControl = new PositionVoltage(0);
  private VelocityVoltage velocityControl = new VelocityVoltage(0);

 // Simulation
 private final SingleJointedArmSim armSim;
 // Feedforward
 private final ArmFeedforward feedforward = new ArmFeedforward(
   0, // kS
   0, // kG
   0, // kV
   0  // kA
 );

  public ArmModuleSim(int id, String bus) {

    this.motor = new TalonFX(id, bus);

    config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = 75;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT;

    // Apply initial configuration with retry logic
    motor.getConfigurator().apply(config, 0.25);

    // Get references to sensor signals
    relativePosition = motor.getPosition();
    motorVelocity = motor.getVelocity();
    motorAppliedVolts = motor.getMotorVoltage();
    motorCurrent = motor.getStatorCurrent();

    // Optimize CAN bus usage
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, relativePosition, motorVelocity, motorAppliedVolts, motorCurrent);
    ParentDevice.optimizeBusUtilizationForAll(motor);


    double armLength = 0.5;
      // Initialize simulation
      armSim = new SingleJointedArmSim(
        DCMotor.getKrakenX60(1), // Motor type
        config.Feedback.SensorToMechanismRatio,
        SingleJointedArmSim.estimateMOI(armLength, 5), // Arm moment of inertia
        armLength, // Arm length (m)
        Units.degreesToRadians(0), // Min angle (rad)
        Units.degreesToRadians(1.5707963267948966), // Max angle (rad)
        true, // Simulate gravity
        Units.degreesToRadians(0) // Starting position (rad)
      );
  }

  @Override
  public void updateInputs(ArmModuleIOInputs inputs) {
       // Set input voltage from motor controller to simulation
   armSim.setInput(motor.getMotorVoltage().getValueAsDouble());
   
   // Update simulation by 20ms
   armSim.update(0.020);
    var motorStatus =
        BaseStatusSignal.refreshAll(
            relativePosition, motorVelocity, motorAppliedVolts, motorCurrent);

    inputs.connected = connectedDebounce.calculate(motorStatus.isOK());
    inputs.positionRad = Units.rotationsToRadians(relativePosition.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(motorVelocity.getValueAsDouble());
    inputs.appliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.currentAmps = motorCurrent.getValueAsDouble();

    if (inputs.appliedVolts > 0) {
      inputs.state = ArmState.FORWARD;
    } else if (inputs.appliedVolts < 0) {
      inputs.state = ArmState.REVERSE;
    } else if (inputs.appliedVolts == 0) {
      inputs.state = ArmState.IDLE;
    } else {
      inputs.state = ArmState.UNKNOWN;
    }
  }

  public void runArm(double positionInRotations,double velocity) {
    double ffVolts = feedforward.calculate(Units.rotationsToRadians(motor.getPosition().getValueAsDouble()),velocity);
    motor.setControl(positionControl.withPosition(positionInRotations).withFeedForward(ffVolts));
    Logger.recordOutput("/Arm/positionInRotations", positionInRotations);
  }

  public void runArm(double positionInRotations) {
    runArm(positionInRotations,0);
    Logger.recordOutput("/Arm/positionInRotations", positionInRotations);
  }

  public void stopArm() {
    runArm(0);
    motor.stopMotor();
    Logger.recordOutput("/Arm/velocityRotPerSec", 0);
  }
}
