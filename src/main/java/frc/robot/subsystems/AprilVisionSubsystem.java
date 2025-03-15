// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.google.gson.Gson;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class AprilVisionSubsystem extends SubsystemBase {
    Coordinate coordinate = new Coordinate();

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tableEntry = table.getEntry("json");
    int fidLocation;
    boolean fidLocFound;
    public AprilVisionSubsystem() {}
    Gson gson = new Gson();
    public enum ReturnTarget{
        TARGET,
        ROBOT,
        FIELD
    }

    double ShootDist = Constants.Shooter.shootingDistance;
    double ShootOff = Constants.Shooter.shootingDistanceOffset;

    public Coordinate getCoordinates(int[] ids, ReturnTarget rt){
        for(int i = 0; i < ids.length; i++){
            getCoordinates(ids[i], rt);
            if(coordinate.aprilTagVisible){
                return coordinate;
            }
        }
        return coordinate;
    }

    public Coordinate getCoordinates(int targetId, ReturnTarget rt) {
       switch(rt){
        case TARGET:
        updateTargetCoordinates(targetId);
        break;
        case ROBOT:
        updateRobotCoordinates(targetId);
        break;
        case FIELD:
        updateFieldCoordinates(targetId);
        break; 
       }
        return coordinate;
    }
    /*public double straightDistance(int id){
        double limeAngleOffset = Math.PI/6;
        getCoordinates(id, ReturnTarget.TARGET);

    }*/

    private void updateTargetCoordinates(int targetId) {
        String jsonString = tableEntry.getString("");
        limelightjson fetchedFiducial = gson.fromJson(jsonString, limelightjson.class);
        if(fetchedFiducial != null){
        if (fetchedFiducial.Fiducial.length != 0) {
             for (int i = 0; i < fetchedFiducial.Fiducial.length; i++) {
                if(fetchedFiducial.Fiducial[i].fID == targetId){
                    fidLocation = i;
                    fidLocFound = true;
                    break;
                }else{
                    fidLocFound = false;
                }
            }if(!fidLocFound){
                //System.out.println("TARGET FIDUCIAL NOT FOUND!");
                SmartDashboard.putBoolean("Fiducial Found", false);
                coordinate.aprilTagVisible = false;
            }else{
                coordinate.x = fetchedFiducial.Fiducial[fidLocation].t6t_rs[0];
                coordinate.y = fetchedFiducial.Fiducial[fidLocation].t6t_rs[1];
                coordinate.z = fetchedFiducial.Fiducial[fidLocation].t6t_rs[2];
                coordinate.rx = fetchedFiducial.Fiducial[fidLocation].t6t_rs[3];
                coordinate.ry = fetchedFiducial.Fiducial[fidLocation].t6t_rs[4];
                coordinate.rz = fetchedFiducial.Fiducial[fidLocation].t6t_rs[5];
                coordinate.aprilTagVisible = true;
                SmartDashboard.putBoolean("Fiducial Found", true);
            } 
        }else{
            coordinate.aprilTagVisible = false;
            SmartDashboard.putBoolean("Fiducial Found", false);
        }
    }else{
         coordinate.aprilTagVisible = false;
         //System.out.println("Fiducial is null");
    }
    }
    private void updateRobotCoordinates(int targetId) {
        String jsonString = tableEntry.getString("");
        limelightjson fetchedFiducial = gson.fromJson(jsonString, limelightjson.class);
        if(fetchedFiducial != null){
        if (fetchedFiducial.Fiducial.length != 0) {
             for (int i = 0; i < fetchedFiducial.Fiducial.length; i++) {
                if(fetchedFiducial.Fiducial[i].fID == targetId){
                    fidLocation = i;
                    fidLocFound = true;
                    break;
                }else{
                    fidLocFound = false;
                }
            }if(!fidLocFound){
                SmartDashboard.putBoolean("Fiducial Found", false);
                coordinate.aprilTagVisible = false;
            }else{
                coordinate.x = fetchedFiducial.Fiducial[fidLocation].t6r_ts[0];
                coordinate.y = fetchedFiducial.Fiducial[fidLocation].t6r_ts[1];
                coordinate.z = fetchedFiducial.Fiducial[fidLocation].t6r_ts[2];
                coordinate.rx = fetchedFiducial.Fiducial[fidLocation].t6r_ts[3];
                coordinate.ry = fetchedFiducial.Fiducial[fidLocation].t6r_ts[4];
                coordinate.rz = fetchedFiducial.Fiducial[fidLocation].t6r_ts[5];
                coordinate.aprilTagVisible = true;
                SmartDashboard.putBoolean("Fiducial Found", true);
            } 
        }else{
            coordinate.aprilTagVisible = false;
            SmartDashboard.putBoolean("Fiducial Found", false);
        }
    }else{
         coordinate.aprilTagVisible = false;
         //System.out.println("Fiducial is null");
    }
    }
    private void updateFieldCoordinates(int targetId) {
        String jsonString = tableEntry.getString("");
        limelightjson fetchedFiducial = gson.fromJson(jsonString, limelightjson.class);
        if(fetchedFiducial != null){
        if (fetchedFiducial.Fiducial.length != 0) {
                coordinate.x = fetchedFiducial.Fiducial[0].t6r_fs[0];
                coordinate.y = fetchedFiducial.Fiducial[0].t6r_fs[1];
                coordinate.z = fetchedFiducial.Fiducial[0].t6r_fs[2];
                coordinate.rx = fetchedFiducial.Fiducial[0].t6r_fs[3];
                coordinate.ry = fetchedFiducial.Fiducial[0].t6r_fs[4];
                coordinate.rz = fetchedFiducial.Fiducial[0].t6r_fs[5];
                coordinate.aprilTagVisible = true;
                SmartDashboard.putBoolean("Fiducial Found", true);
        }else{
            coordinate.aprilTagVisible = false;
            SmartDashboard.putBoolean("Fiducial Found", false);
        }
    }else{
         coordinate.aprilTagVisible = false;
         //System.out.println("Fiducial is null");
    }

    }
    @Override
    public void periodic() {
        Coordinate position = this.getCoordinates(4,AprilVisionSubsystem.ReturnTarget.TARGET);
        System.out.println(position);
        double distance = position.z;
    if ((ShootDist - ShootOff <= distance) && (distance <= ShootDist + ShootOff)) { //green
        SmartDashboard.putBoolean("In Shooting Distance", true);
    }else{//red
        SmartDashboard.putBoolean("In Shooting Distance", false);
    }
    }

public class Coordinate {
    public double x;
    public double y;
    public double z;
    public double rx;
    public double ry;
    public double rz;
    public boolean aprilTagVisible;
}

class limelightjson{
    //public ResultJson Results;
    public FiducialJson[] Fiducial;
}

class ResultJson
{
    public FiducialJson[] Fiducial;
    public int pID;
    public double tl;
    public double ts;
    public int v;
}

class FiducialJson
{
    public int fID;
    public String fam;
    public double[] t6t_rs;
    public double[] t6r_ts;
    public double[] t6r_fs;
}
}