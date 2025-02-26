package frc.robot.subsystems.Motors;

public interface GenericMotor {
    void set(double output);  // Open-loop control
    void setVoltage(double voltage);  // Voltage control
    void setPosition(double rotations);  // Closed-loop position control

    double getPosition();  // Relative position (internal encoder)
    double getVelocity();  // Velocity in RPM
    double getAbsolutePosition();  // Absolute position from external encoder
}

