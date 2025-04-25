// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Motors;

/** Add your docs here. */
public class SIMMotor implements GenericMotor{
    // dummy class
    public SIMMotor(int canID){
        return;
    }
    @Override
    public void set(double output) {
        return;
    }

    @Override
    public void setVoltage(double voltage) {
        return;
    }

    @Override
    public void setPosition(double rotations) {
        //TODO: ADD PID LOGIC HERE
        // Use the internal PID to move to position
        return;
    }

    @Override
    public double getPosition() {
        return 0.0d;  // Returns relative encoder position (resets at power cycle)
    }

    @Override
    public double getVelocity() {
        return 0.0d;  // Returns velocity in RPM
    }

    @Override
    public double getAbsolutePosition() {
        return 0.0d;  // Returns absolute encoder position (persists across power cycles)
    }

    @Override
    public void overridePosition(double rotations) {
        return;
    }
}