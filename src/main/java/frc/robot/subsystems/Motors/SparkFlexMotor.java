package frc.robot.subsystems.Motors;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkFlexMotor extends SubsystemBase implements GenericMotor {
    private final SparkFlex motor;
    private final RelativeEncoder relativeEncoder;
    private final AbsoluteEncoder absoluteEncoder;

    int m_canId;

    SparkFlexConfig pidConfig;

    SparkClosedLoopController sparkPid;

    public SparkFlexMotor(int canID) {
        m_canId = canID;
        motor = new SparkFlex(canID, SparkFlex.MotorType.kBrushless);
        relativeEncoder = motor.getEncoder();
        absoluteEncoder = motor.getAbsoluteEncoder();
    }

    public SparkFlexMotor(int canID, double kP, double kI, double kD, double minPower, double maxPower) {
        motor = new SparkFlex(canID, MotorType.kBrushless);

        pidConfig.closedLoop
            .p(kP)
            .i(kI)
            .d(kD)
            .outputRange(minPower, maxPower);

        sparkPid = motor.getClosedLoopController();

        motor.configure(pidConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        relativeEncoder = motor.getEncoder();
        absoluteEncoder = motor.getAbsoluteEncoder();
    }

    @Override
    public void set(double output) {
        motor.set(output);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setPosition(double rotations) {
        //TODO: ADD PID LOGIC HERE
        // Use the internal PID to move to position
        
    }

    @Override
    public double getPosition() {
        return relativeEncoder.getPosition();  // Returns relative encoder position (resets at power cycle)
    }

    @Override
    public double getVelocity() {
        return relativeEncoder.getVelocity();  // Returns velocity in RPM
    }

    @Override
    public double getAbsolutePosition() {
        return absoluteEncoder.getPosition();  // Returns absolute encoder position (persists across power cycles)
    }

    @Override
    public void overridePosition(double rotations) {
        relativeEncoder.setPosition(rotations);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("MotorOutputs/MOTOR_" + m_canId + "_Output", motor.get());
    }
}
