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

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  //TODO: CHECK AND PROGRAM ALL IDS ACCORDING TO THIS CONSTANTS FILE!!!!

  public class OperatorConstants{
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kBluetoothControllerPort = 2; 
  }
  public class DriveMotors {
    public class FrontLeft {
      public static final int driveId = 1;
      public static final int steerId = 2;
      public static final int encoderId = 3;

    }
    public class FrontRight {
      public static final int driveId = 4;
      public static final int steerId = 5;
      public static final int encoderId = 6;
    }
    public class BackLeft {
      public static final int driveId = 7;
      public static final int steerId = 8;
      public static final int encoderId = 9;
    }
    public class BackRight {
      public static final int driveId = 10;
      public static final int steerId = 11;
      public static final int encoderId = 12;
      }
  }

  public class AgitatorMotors {
    public static final int rotateId = 13;
    public static final int wheelId = 14;
    public static final int elevatorId = 15;
    public static final double elevatorInitialPosition = 0.0; //MAY NEED UPDATED?
    //TODO: FINAL POSITION
    public static final double elevatorFinalPosition = 0.1; // safe value
    public static final double elevatorMaxMarginOfError = Math.abs(elevatorFinalPosition - elevatorInitialPosition); // to avoid making it angry, abs
    
    public class ElevatorPid{
      public static final double maxPower = 0.25;
      public static final double tolerance = 0.05;
      public static final double kP = maxPower / elevatorMaxMarginOfError;
      public static final double kI = 0.0;
      public static final double kD= 0.0;
    }
    public static final double rotatorInitialPosition = 0.0; //MAY NEED UPDATED?
    //TODO: FINAL POSITION
    public static final double rotatorFinalPosition = 0.1; // safe value
    public static final double rotatorMaxMarginOfError = Math.abs(rotatorFinalPosition - rotatorInitialPosition); // to avoid making it angry, abs
    
    public class RotatorPid{
      public static final double maxPower = 0.25;
      public static final double tolerance = 0.05;
      public static final double kP = maxPower / rotatorMaxMarginOfError;
      public static final double kI = 0.0;
      public static final double kD= 0.0;
    }
  }

  public class ClimbMotors {
    public static final int climb = 16; //change back to 16!!!
    public static final int winch = 22; // TODO: find actual CANID
    public static double climbSpeed = 0.05;
    public static final double winchSpeed = 1;
    public static final double winchOutSpeed = 0.2;
    public static final int climbGearRatio = 80; //80:1
  }

  // TODO: Limelight Notes:
  // Limelight angle: 32 degrees up
  // Limelight up: 21.25 inches above ground
  // Limelight left: 9.25 inches left
  // Limelight out unadjusted: 12.5 inches forwards
  // robot is 14.5 inches wide along length
  // set it at 0 point to get distance from limelight
  // then subtract 2 to get distance from edge
  // in code
    public class IntakeMotors{
      public static final int pivotId = 17;
      public static final int pivotGearboxRatio = 17;
      public static final double pivotInitialPosition = -0.071428; //MAY NEED UPDATED?
      //TODO: FINAL POSITION
      public static final double pivotFinalPosition = -2.7; // should be -3.261902332305908, but code angry when final position negative
      public static final double maxMarginOfError = Math.abs(pivotFinalPosition - pivotInitialPosition); // to avoid making it angry, abs
      
    public class PivotPid{
      
      public static final double maxPower = 0.4;
      public static final double tolerance = 0.1;
      public static final double kP = maxPower / maxMarginOfError;
      public static final double kI = 0.0;
      public static final double kD= kP/2;
    }
    public static final int rollerId = 18;
    public static final double defaultRollerSpeed = 0.25;
  }

  public class Shooter{ 
    public static final int greenRollerId = 19;
    public static final int frontRollerId = 20;
    public static final int backRollerId = 21;
    public static final double defaultSpeed = 0.40;
    public static final double vomitSpeed = -0.1;
    public static final double rollerSpeed = 0.40;
    public static final double speedIncrement = 0.0;
    public static final double limelightOffset = -0.3683; // offset to get distance from front edge. Currently half the robot length

    public class ShooterPid{
      public static final double kP = 0.08;
      public static final double kI = 0.001;
      public static final double kD = 3.0;
      public static final double tolerance = 0.1;     
    }
  }
}
