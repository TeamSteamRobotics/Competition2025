// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Test;

<<<<<<< HEAD:src/main/java/frc/robot/commands/Test/Add.java
import frc.robot.subsystems.Test.ButtonTest;
=======
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
>>>>>>> fc28bc1793cf13a3a7e5145fa6210a2fb7f53522:src/main/java/frc/robot/commands/ButtonTest.java
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Add extends Command {
  /** Creates a new Add. */

  private ButtonTest test;

  public Add(ButtonTest k_test) {
    test = k_test;
    addRequirements(test);
  }

  public static void Calculate() {
    double test = SmartDashboard.getNumber("test", -1);
    SmartDashboard.putNumber("test_add", test + 2);
    SmartDashboard.putNumber("test_subtract", test - 2);
    SmartDashboard.putNumber("test_multiply", test * 2);
    SmartDashboard.putNumber("test_divide", test / 2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    test.add();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    test.stopButtonTest();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
