package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Shoot extends Command {
    
    ShooterSubsystem shooterSubsystem;
    double distance;
    double speed;

    public Shoot(ShooterSubsystem s_shooterSubsystem, double s_distance) {

        shooterSubsystem = s_shooterSubsystem;
        distance = s_distance;
        addRequirements(s_shooterSubsystem);
    }


    public void initialize(){

    }

    public void execute(){
        shooterSubsystem.setShooterSpeedPID(1);

    }

    public void end(boolean interrupted){

    }

    public boolean isFinished(){
            return false;

    }
}