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
  public static class AgitatorConsts {
    public static int AgitatorPivotID = 13;
    public static int AgitatorWheelID = 14;
    public static int AgitatorElevatorID = 15;
    public static double AgitatorRaiseSpeed = 0.2; // placeholder speeds
    public static double AgitatorWheelSpinSpeed = 0.2;
    public static double AgitatorRotateSpeed = 0.2;
    // Want to raise by each level of algae.
    public static double AgitatorLevelDistance = 1; //TODO: Calculate number rotations required to get to each level of agitator.
    //12 to 1 ratio
    
    // for commands
    public static double AgitatorMaxHeight = 1; //TODO: Set atual value; Need agitator for this
  }
}
