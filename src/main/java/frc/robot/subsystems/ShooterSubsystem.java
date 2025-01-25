package frc.robot.subsystems;

import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.config.*;

public class ShooterSubsystem extends SubsystemBase{

    private SparkFlex shooter;

    public ShooterSubsystem(){

        shooter = new SparkFlex(1, SparkLowLevel.MotorType.kBrushless);
    }

    public void setShooterSpeedPID(int i) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setShooterSpeedPID'");
    }
}

