package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

public class VisionSubsystem extends SubsystemBase {

  public class FusedFiducialType {
    public LimelightHelpers.LimelightTarget_Fiducial processedAprilTag;
    public RawFiducial rawAprilTag;
    public FusedFiducialType() {
      processedAprilTag = null;
      rawAprilTag = null;
    }
    // there is more then one data type and this is why we are packing them together here
    // It was exceedingly inconvenient to have to use both of them in different places
  } 
  public FusedFiducialType[] AprilTags; // List of all apriltags on the field
  // Seen ones are non-null, indexed by Apriltag ID - 1
  
  public VisionSubsystem() {
    AprilTags = new FusedFiducialType[22];
    LimelightHelpers.setPipelineIndex("", 0); // 0 is 2D Fiducials, 1 is 3D fiducials
  }


  @Override
  public void periodic() {
    LimelightHelpers.LimelightTarget_Fiducial[] temp = LimelightHelpers.getLatestResults("").targets_Fiducials;
    for(int i = 0; i < 22; i++){
      AprilTags[i] = null; // You make me very unhappy. Please stop it. What could go wrong?
    }
    if(!LimelightHelpers.getTV("")){
      return;
    } // Early return to skip the below code if there is no valid target. Shouldn't be important, but may as well have it.
    for(int i = 0; i < temp.length; i++)
    {
      
      AprilTags[(int) temp[i].fiducialID - 1].processedAprilTag = temp[i]; // Reorders the temp array such that all Apriltags are in the array index correspondingly to their ID
      // If Apriltag data is ending up in the wrong place, use Math.round before typecasting
      // Wrong place specifically being 1 slot before it should be
      // If Java C-style typecasting works like C typecasting, it's weird.
      AprilTags[(int) temp[i].fiducialID - 1].rawAprilTag = LimelightHelpers.getRawFiducials("")[i];
    }
    // This method will be called once per scheduler run
  }
  
  public double[] getFiducialDistanceToCamera() 
  {
    // this is supposed to return the ordered distances to the Apriltags as an array
    // indexed by AprilTag ID - 1
    double[] orderedDistances = new double[22];
    for(int i = 0; i < 22; i++)
    {
      if(AprilTags[i] == null){
        continue; // Avoids errors for unseen apriltags
      }
      orderedDistances[i] = AprilTags[i].rawAprilTag.distToCamera;
    }
    
    return orderedDistances;
  }
  public double getGivenFiducialDistance(int id){
    if(id < 0 || id > 21){
      return -1; //Out of Bounds
    }
    if(AprilTags[id] == null){
      return -1; //Apriltag not seen
    }
    double rawDistance = getFiducialDistanceToCamera()[id];
    double hOffset = AprilTags[id].processedAprilTag.tx;
    double vOffset = AprilTags[id].processedAprilTag.ty; // angle offsets
    double horizontalAdjustedDistance = rawDistance * Math.cos(Math.toRadians(hOffset));
    double finalApproxDist = horizontalAdjustedDistance * Math.cos(Math.toRadians(vOffset));
    return finalApproxDist; // TODO: [pray]
  }
  //SmartDashboard.putNumber("Limelight Distance", orderedDistances);
}
