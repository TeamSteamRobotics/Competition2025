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
  public class DriveMotors {
    public class FrontLeft {
      public static final int driveId = 1;
      public static final int steerId = 2;
    }
    public class FrontRight {
      public static final int driveId = 3;
      public static final int steerId = 4;
    }
    public class BackLeft {
      public static final int driveId = 5;
      public static final int steerId = 6;
    }
    public class BackRight {
      public static final int driveId = 7;
      public static final int steerId = 8;
      }
  }

  public class AgitatorMotors {
    public static final int rotateId = 9;
    public static final int wheelId = 10;
    public static final int elevatorId = 11;
  }

  public class ClimbMotors {
    public static final int climb = 12;
  }

  public class IntakeMotors{
    public static final int pivotId = 13;
    public static final int pivotGearboxRatio = 9;
    public static final double pivotInitialPosition = 0.0;
    //TODO: FINAL POSITION
    public static final double pivotFinalPosition = 0.0;

    public class PivotPid{
      public static final double tolerance = 0.5;
      public static final double kP = 0.5;
      public static final double kI = 0.001;
      public static final double kD= 3.0;
    }
    public static final int rollerId = 14;
    public static final double defaultRollerSpeed = 0.45;
  }

  public class Shooter{ 
    public static final int frontRollerId = 15;
    public static final int backRollerId = 16;
    public static final double defaultSpeed = 0.35;
    public static final double speedIncrement = 0.1;

    public class ShooterPid{
      public static final double kP = 0.08;
      public static final double kI = 0.001;
      public static final double kD = 3.0;
      public static final double tolerance = 0.1;     
    }
  }
}
