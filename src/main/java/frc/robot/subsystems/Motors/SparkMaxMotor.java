package frc.robot.subsystems.Motors;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkMaxMotor implements GenericMotor {
    private final SparkMax motor;
    private final RelativeEncoder relativeEncoder;
    private final AbsoluteEncoder absoluteEncoder;
    SparkClosedLoopController sparkPid;

    SparkMaxConfig pidConfig;

    public SparkMaxMotor(int canID) {
        motor = new SparkMax(canID, MotorType.kBrushless);
        relativeEncoder = motor.getEncoder();
        absoluteEncoder = motor.getAbsoluteEncoder();
    }

    public SparkMaxMotor(int canID, double kP, double kI, double kD, double minPower, double maxPower) {
        motor = new SparkMax(canID, MotorType.kBrushless);

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
        sparkPid.setReference(0, ControlType.kDutyCycle);
        motor.set(output);
    }

    @Override
    public void setVoltage(double voltage) {
        sparkPid.setReference(0, ControlType.kVoltage);
        motor.setVoltage(voltage);
    }

    @Override
    public void setPosition(double rotations) {
        sparkPid.setReference(rotations, ControlType.kPosition);
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
}
