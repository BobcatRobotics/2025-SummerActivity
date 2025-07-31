package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsystem extends SubsystemBase {
    private final TalonFX m_rollerMotor = new TalonFX(0, "rio"); // Assuming CAN ID 0
    private final VelocityDutyCycle m_velocityRequest = new VelocityDutyCycle(0);

    public RollerSubsystem() {
       TalonFXConfiguration config = new TalonFXConfiguration();
       config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
       config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
       config.CurrentLimits.StatorCurrentLimit = 20;
       config.CurrentLimits.StatorCurrentLimitEnable = true;
       var Slot0Configs = new Slot0Configs();
       Slot0Configs.kP = 0.1;
       config.Slot0 = Slot0Configs;
    

       m_rollerMotor.getConfigurator().apply(config);
    }
    @Override
    public void periodic() {
       
    }

    public void spin_roller(double output) {
        m_velocityRequest.withVelocity(output);
        m_rollerMotor.setControl(m_velocityRequest);
    }

    public void stop_motor() {
        m_rollerMotor.stopMotor();
    }
}
