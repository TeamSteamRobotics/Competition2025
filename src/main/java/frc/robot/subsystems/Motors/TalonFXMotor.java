package frc.robot.subsystems.Motors;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;

public class TalonFXMotor implements GenericMotor {
    private final TalonFX motor;
    private CANBus m_canBus;
    
    public TalonFXMotor(int canID, String canBus) {
        m_canBus = new CANBus(canBus);
        motor = new TalonFX(canID, m_canBus);
        // Configure PID or settings if needed
        TalonFXConfiguration config = new TalonFXConfiguration();
        motor.getConfigurator().apply(config);
    }

    @Override
    public void set(double output) {
        motor.setControl(new DutyCycleOut(output));
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setControl(new VoltageOut(voltage));
    }

    @Override
    public void setPosition(double rotations) {
        motor.setControl(new PositionVoltage(rotations));
    }

    @Override
    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    @Override
    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    @Override
    public double getAbsolutePosition() {
        //FIXME: REQUIRES ADDITIONAL SETUP ON TALONFX HARDWARE
       return motor.getPosition().getValueAsDouble();
    }
}