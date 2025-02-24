package frc.robot.subsystems.Motors;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;

public class SparkFlexMotor implements GenericMotor {
    private final SparkFlex motor;
    private final RelativeEncoder relativeEncoder;
    private final AbsoluteEncoder absoluteEncoder;

    public SparkFlexMotor(int canID) {
        motor = new SparkFlex(canID, SparkFlex.MotorType.kBrushless);
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
        // Use the internal PID to move to position
        relativeEncoder.setPosition(rotations);
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
}
