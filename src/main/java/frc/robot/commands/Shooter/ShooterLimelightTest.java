// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Shooter;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.VisionSubsystem;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class ShooterLimelightTest extends Command {
//   PrimeShooter cmdShooter;
//   VisionSubsystem m_vis;
//   /** Creates a new ShooterLimelightTest. */
//   public ShooterLimelightTest(ShooterSubsystem shooter, double speed, VisionSubsystem vision) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     cmdShooter = new PrimeShooter(shooter, speed);
//     m_vis = vision;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {}

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     double distance = m_vis.getFiducialDistanceToCamera()[4]; // curse you zero-indexing in this one particular case
//     SmartDashboard.putNumber("Distance", (distance != 0) ? distance : -1d ); // should fetch the distance to Apriltag 5 if it is seen, and give -1 otherwise
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
