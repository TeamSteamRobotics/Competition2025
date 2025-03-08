// package frc.robot.subsystems;

// import java.util.Map;
// import java.util.TreeMap;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.LimelightHelpers;
// import frc.robot.LimelightHelpers.RawFiducial;

// public class VisionSubsystem extends SubsystemBase {

//    public RawFiducial[] fiducials;
//    TreeMap<Integer, processedAprilTag> AprilTags;

//   class processedAprilTag{
//     int id;
//     double distToCamera;
//     double distToRobot;
//     double ambiguity;
//     public processedAprilTag(int id, double dC, double dR, double ambig){
//       this.id = id;
//       distToCamera = dC;
//       distToRobot = dR;
//       ambiguity = ambig;
//     }
//   }
//     // there is more then one data type and this is why we are packing them together here
//     // It was exceedingly inconvenient to have to use both of them in different places


//   // List of all apriltags on the field
//   // Seen ones are non-null, indexed by Apriltag ID - 1
  
//   public VisionSubsystem() {
//     AprilTags = new TreeMap<Integer, processedAprilTag>();
//     LimelightHelpers.setPipelineIndex("", 1); 
    
//     // 0 is 2D Fiducials, 1 is 3D fiducials
//   }


//   @Override
//   public void periodic() {
//     fiducials = LimeLi

//     for(int i = 0; i < fidi){
//       AprilTags.put(fiducial.id, new processedAprilTag(fiducial.id, fiducial.distToCamera, fiducial.distToRobot, fiducial.ambiguity));
//     }

//     //try{
//     //  LimelightHelpers.LimelightTarget_Fiducial[] temp = LimelightHelpers.getLatestResults("").targets_Fiducials;
   
//     //LimelightHelpers.LimelightTarget_Fiducial[] temp = LimelightHelpers.getLatestResults("").targets_Fiducials;
    
//     // Early return to skip the below code if there is no valid target. Shouldn't be important, but may as well have it.
    
//     // This method will be called once per scheduler run
//   }
  
//   public double getFiducialDistanceToCamera(int id) 
//   {
//     if(!AprilTags.containsKey(id)){
//       return -1;
//     }
//     return AprilTags.get(id).distToCamera;
//   }
//   public double getGivenFiducialDistance(int id){
//     LimelightHelpers.getLatestResults("limelight").targets_Fiducials[]
    
//   /*   if(id < 0 || id > 21){
//       return -1; //Out of Bounds
//     }
    
//     if(AprilTags.get(id) == null){
//       return -1; //Apriltag not seen
//     }
    
//     double rawDistance = getFiducialDistanceToCamera(id);
//     double hOffset = AprilTags.get(id);
//     double vOffset = AprilTags[id].processedAprilTag.ty; // angle offsets
//     double horizontalAdjustedDistance = rawDistance * Math.cos(Math.toRadians(hOffset));
//     double finalApproxDist = horizontalAdjustedDistance * Math.cos(Math.toRadians(vOffset));
//     return finalApproxDist; // TODO: [pray]
//   }
//   //SmartDashboard.putNumber("Limelight Distance", orderedDistances);*/
// }
// }
