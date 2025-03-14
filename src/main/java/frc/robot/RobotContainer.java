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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PathFind;
import frc.robot.commands.Agitator.AgitatorStateMachine;
import frc.robot.commands.Climb.ClimbIn;
import frc.robot.commands.Climb.ClimbOut;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AgitatorSubsystem;
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
//import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
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
  private final AgitatorSubsystem m_agit;
  //private final VisionSubsystem vision;
  
  // Controllers
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  //private final CommandXboxController operator = new CommandXboxController(1);

  private final Trigger intakePivot = m_operatorController.leftTrigger();
  private final Trigger intakeRollers = m_operatorController.leftTrigger(0.80);
  private final Trigger vomit = m_operatorController.a();
  private final Trigger climbOut = m_driverController.leftBumper();
  private final Trigger climbIn = m_driverController.rightBumper();
  private final Trigger shooterRollers = m_operatorController.rightTrigger();
  private final Trigger greenRollers = m_operatorController.b();
  private final Trigger putCoralToggle = m_operatorController.leftBumper();
  private final Trigger getCoralToggle = m_operatorController.rightBumper();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   // SmartDashboard.getnum

    m_intake = new IntakeSubsystem();
    m_shooter = new ShooterSubsystem();
    m_climb = new ClimbSubsystem();
    m_agit = new AgitatorSubsystem();
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
    // Pathplanner command registering
    NamedCommands.registerCommand("IntakeDeploy", new Pivots(m_intake, Constants.IntakeMotors.pivotFinalPosition));
    NamedCommands.registerCommand("IntakeRetract", new Pivots(m_intake, Constants.IntakeMotors.pivotInitialPosition));
    //TODO: NamedCommands.registerCommand("RollerIn", new Roll(intake, Constants.IntakeMotors.defaultRollerSpeed));
    //TODO: NamedCommands.registerCommand("IntakeOut", new Roll(intake, -Constants.IntakeMotors.defaultRollerSpeed));

    NamedCommands.registerCommand("ShooterDefault", new PrimeShooter(m_shooter, Constants.Shooter.defaultSpeed));
    NamedCommands.registerCommand("ShooterDistance (UNIMPLEMENTED)", new PrimeShooter(m_shooter, /*TODO:CHANGE TO DISTANCE SENSOR*/null));

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
    climbOut.onTrue(new ClimbOut(m_climb));

    //Climb in
    climbIn.whileTrue(new ClimbIn(m_climb));

    // Intake out
    intakePivot.onTrue(new Pivots(m_intake, Constants.IntakeMotors.pivotFinalPosition));  

    // Intake in
    intakePivot.onFalse(new Pivots(m_intake, Constants.IntakeMotors.pivotInitialPosition));  

    // Roll intake wheels
    intakeRollers.whileTrue(new Roll(m_intake, Constants.IntakeMotors.defaultRollerSpeed));

    // VomitButton
    vomit.whileTrue(new Roll(m_intake, -Constants.IntakeMotors.defaultRollerSpeed));

    // Rev shooter rollers
    shooterRollers.whileTrue(new PrimeShooter(m_shooter, Constants.Shooter.defaultSpeed));

    // Run green rollers
    greenRollers.onTrue(new RollGreen(m_shooter, Constants.Shooter.defaultSpeed, true));

    putCoralToggle.onTrue(AgitatorStateMachine.transitState("PUT_CORAL", m_agit));
    getCoralToggle.onTrue(AgitatorStateMachine.transitState("GET_CORAL", m_agit));

    //operator.a().toggleOnTrue(new PrimeShooter(shooter, /*TODO:CHANGE TO DISTANCE SENSOR*/null));
    //.m_driverController.b().toggleOnTrue(new PrimeShooter(shooter, () -> shooter.lookupShootSpeed(vision.getGivenFiducialDistance(3)))); // dam zero-indexing
 
    // operator.povUp().whileTrue(new RepeatCommand(new InstantCommand(() -> shooter.ShootPID(shooter.getTargetSpeed() + Constants.Shooter.speedIncrement))));
    // operator.povDown().whileTrue(new RepeatCommand(new InstantCommand(() -> shooter.ShootPID(shooter.getTargetSpeed() - Constants.Shooter.speedIncrement))));
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
