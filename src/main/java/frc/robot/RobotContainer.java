// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PathFind;
import frc.robot.commands.StopMotors;
import frc.robot.commands.Climb.RetractClimb;
import frc.robot.commands.Climb.RetractWinch;
import frc.robot.commands.Climb.RaiseClimb;
import frc.robot.commands.Climb.RaiseWinch;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AprilVisionSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.commands.Intake.Pivots;
import frc.robot.commands.Intake.Roll;
import frc.robot.commands.Intake.Tests.PivotTest;
import frc.robot.commands.Shooter.PrimeShooter;
import frc.robot.commands.Shooter.RollGreen;
//import frc.robot.commands.Shooter.ShooterLimelightTest;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.AprilVisionSubsystem.Coordinate;
import frc.robot.subsystems.AprilVisionSubsystem.ReturnTarget;
//import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.commands.PathPlanner.GreenBeambreak;
import frc.robot.commands.PathPlanner.RaiseClimbPathplanner;
import frc.robot.commands.PathPlanner.StartGreen;
import frc.robot.commands.PathPlanner.StartOrange;
import frc.robot.commands.PathPlanner.StopGreen;
import frc.robot.commands.PathPlanner.StopOrange;
import frc.robot.commands.PathPlanner.StartBlack;
import frc.robot.commands.PathPlanner.StopBlack;

import java.util.function.Supplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final IntakeSubsystem m_intake;
  private final ShooterSubsystem m_shooter;
  private final ClimbSubsystem m_climb;
  private final AprilVisionSubsystem m_vision;
  //private final VisionSubsystem vision;
  
  // Controllers
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  //private final CommandXboxController operator = new CommandXboxController(1);

  //private final Trigger intakePivot = m_operatorController.leftTrigger();
  private final Trigger intakeRollers = m_operatorController.leftTrigger(0.80);
  private final Trigger intakePivot = m_operatorController.y();
  private final Trigger vomit = m_operatorController.povLeft();
  private final Trigger intakeOut = m_operatorController.a();
  private final Trigger climbOut = m_driverController.leftTrigger();
  private final Trigger climbIn = m_driverController.rightTrigger();
  private final Trigger shooterRollers = m_operatorController.rightTrigger();
  private final Trigger greenRollers = m_operatorController.b();
  private final Trigger shooterDistanceRollers = m_operatorController.x();

  private final Trigger winchIn = m_driverController.rightBumper();
  private final Trigger winchOut = m_driverController.leftBumper();

  //Supplier<Coordinate> coordinateSupplier; // god help me :3

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer()
   {

   // SmartDashboard.getnum

    m_intake = new IntakeSubsystem();
    m_shooter = new ShooterSubsystem();
    m_climb = new ClimbSubsystem();
    m_vision = new AprilVisionSubsystem();


      // Pathplanner command registering
      NamedCommands.registerCommand("IntakeDeploy", new Pivots(m_intake, Constants.IntakeMotors.pivotFinalPosition, "In"));
      NamedCommands.registerCommand("IntakeRetract", new Pivots(m_intake, Constants.IntakeMotors.pivotInitialPosition, "Out"));
      //TODO: NamedCommands.registerCommand("RollerIn", new Roll(intake, Constants.IntakeMotors.defaultRollerSpeed));
      //TODO: NamedCommands.registerCommand("IntakeOut", new Roll(intake, -Constants.IntakeMotors.defaultRollerSpeed));
      //NamedCommands.registerCommand("ShooterDefault", new PrimeShooter(m_shooter, Constants.Shooter.defaultSpeed));
      NamedCommands.registerCommand("StartGreen", new StartGreen(m_shooter, 0.2));
      NamedCommands.registerCommand("StopGreen", new StopGreen(m_shooter));
      NamedCommands.registerCommand("StartOrange", new StartOrange(m_shooter, Constants.Shooter.defaultSpeed));
      NamedCommands.registerCommand("StopOrange", new StopOrange(m_shooter));
      NamedCommands.registerCommand("StartBlack", new StartBlack(m_intake, Constants.IntakeMotors.defaultRollerSpeed));
      NamedCommands.registerCommand("StopBlack", new StopBlack(m_intake));
      NamedCommands.registerCommand("GreenBeambreak", new GreenBeambreak(m_shooter, 0.25));
      NamedCommands.registerCommand("RaiseClimb", new RaiseClimbPathplanner(m_climb));
      //NamedCommands.registerCommand("ShooterDistance (UNIMPLEMENTED)", new PrimeShooter(m_shooter, /*TODO:CHANGE TO DISTANCE SENSOR*/null));

    //coordinateSupplier = () -> m_vision.getCoordinates(new int[]{4, 5}, ReturnTarget.TARGET);
    //vision = new VisionSubsystem();
    switch (Constants.currentMode) {
       
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
        //TODO: Change 'TunerConstants' to 'Constants' (After we update the values on the motors as needed)
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }

  
    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    autoChooser.addOption("Pathfind Test", PathFind.toPose(new Pose2d(1.5, 6, new Rotation2d(0))));
    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX()));

    // Lock to 0° when A button is held 
    m_driverController.a()
         .whileTrue(
             DriveCommands.joystickDriveAtAngle(
                 drive,
                 () -> -m_driverController.getLeftY(),
                 () -> -m_driverController.getLeftX(),
                 () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    m_driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    
     //Reset gyro to 0° when B button is pressed
    m_driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
   
    // Climb out
    climbOut.whileTrue(new RaiseClimb(m_climb));

    //Climb in
    climbIn.whileTrue(new RetractClimb(m_climb));

    //Winch in
    winchIn.whileTrue(new RetractWinch(m_climb));
    //winchIn.whileTrue(new ParallelCommandGroup(new RetractWinch(m_climb)));
    winchOut.whileTrue(new RaiseWinch(m_climb));

    // Intake out
    intakeOut.onTrue(new ParallelCommandGroup(new Pivots(m_intake, Constants.IntakeMotors.pivotFinalPosition, "Out"), new RollGreen(m_shooter, Constants.Shooter.rollerSpeed, false)));  

    // Intake in
    intakePivot.onTrue(new Pivots(m_intake, Constants.IntakeMotors.pivotInitialPosition, "In"));  

    // Roll intake wheels
    intakeRollers.whileTrue(new Roll(m_intake, Constants.IntakeMotors.defaultRollerSpeed));

    // VomitButton
    vomit.whileTrue(new ParallelCommandGroup(new Roll(m_intake, -Constants.IntakeMotors.defaultRollerSpeed), new RollGreen(m_shooter, -Constants.Shooter.rollerSpeed, true)));

    // Rev shooter rollers
    shooterRollers.whileTrue(new PrimeShooter(m_shooter, Constants.Shooter.defaultSpeed));

    // Run green rollers
    greenRollers.whileTrue(new RollGreen(m_shooter, Constants.Shooter.rollerSpeed, true));

    //Run shooter based on distance

    //shooterDistanceRollers.whileTrue(new PrimeShooter(m_shooter, coordinateSupplier.get().aprilTagVisible ? (() -> coordinateSupplier.get().z) : (() -> -1))); // TODO: Make it take  multiple IDs
    // TODO: fix in general

    //operator.a().toggleOnTrue(new PrimeShooter(shooter, /*TODO:CHANGE TO DISTANCE SENSOR*/null));
    //.m_driverController.b().toggleOnTrue(new PrimeShooter(shooter, () -> shooter.lookupShootSpeed(vision.getGivenFiducialDistance(3)))); // dam zero-indexing
    shooterDistanceRollers.whileTrue(new PrimeShooter(m_shooter, () -> (getDistance() + Constants.Shooter.limelightOffset)));
    m_operatorController.y().whileTrue(new RollGreen(m_shooter, Constants.Shooter.rollerSpeed, true)); // TODO: use parallel commands
 
    // operator.povUp().whileTrue(new RepeatCommand(new InstantCommand(() -> shooter.ShootPID(shooter.getTargetSpeed() + Constants.Shooter.speedIncrement))));
    // operator.povDown().whileTrue(new RepeatCommand(new InstantCommand(() -> shooter.ShootPID(shooter.getTargetSpeed() - Constants.Shooter.speedIncrement))));
  }
  public double getDistance(){ // TODO: god help me again :3
    if(m_vision.getCoordinates(new int[]{4, 5}, ReturnTarget.TARGET).aprilTagVisible){ // TODO: add fIDs for other side of barge
        return m_vision.getCoordinates(new int[]{4, 5}, ReturnTarget.TARGET).z;
    }
    return -1;
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void stopMotors(){
    CommandScheduler.getInstance().schedule(new StopMotors(m_climb, m_intake, m_shooter));
  }
}
