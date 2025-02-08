package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {

  public LimelightHelpers.LimelightTarget_Fiducial AprilTag;
  public VisionSubsystem() {
    AprilTag = new LimelightHelpers.LimelightTarget_Fiducial();
    LimelightHelpers.setPipelineIndex("", 0); // 0 is 2D Fiducials, 1 is 3D fiducials
  }


  @Override
  public void periodic() {
    if(!LimelightHelpers.getTV("")){ //getTV returns true if there is a valid target
        AprilTag = null; // TODO: Better error case
        return;
    }
    AprilTag = LimelightHelpers.getLatestResults("").targets_Fiducials[0];
    
    // This method will be called once per scheduler run
  }
}
