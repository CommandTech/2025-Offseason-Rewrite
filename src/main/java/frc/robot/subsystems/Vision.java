// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Paths;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tv = table.getEntry("tv");
  private NetworkTableEntry tl = table.getEntry("tl");
  private double[] botPoseBLUE = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botPose_wpiblue").getDoubleArray(new double[6]);
  private Pose3d botPose3d = new Pose3d(botPoseBLUE[0],botPoseBLUE[1],botPoseBLUE[2],new Rotation3d(botPoseBLUE[3],botPoseBLUE[4],botPoseBLUE[5]));

  //Might move to a vision subsystem if we get to that point
  private AprilTagFieldLayout fieldLayout;
  {
    try {
      fieldLayout = new AprilTagFieldLayout(
          Paths.get("src", "main", "java", "frc", "apriltagmap", "2025-reefscape-welded.json").toAbsolutePath().toString()
      );
      fieldLayout.setOrigin(new Pose3d(0,0,0, new Rotation3d()));
    } catch (IOException e) {
      e.printStackTrace();
      fieldLayout = null;
    }
  }

  public Vision() {   
    CameraServer.startAutomaticCapture();
    CameraServer.getVideo();
  }

  @Override
  public void periodic() {
    updateVisionPose();
  }

  public Pose3d getVisionPose(){
    //Adds the camera offset constant to the bot pose to correct for the location of the camera
    return botPose3d.plus(VisionConstants.CAMERA_OFFSET);
  }

  private void updateVisionPose(){
    if (tv.getBoolean(false)){
      botPoseBLUE = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botPose_wpiblue").getDoubleArray(new double[6]);
      botPose3d = new Pose3d(botPoseBLUE[0],botPoseBLUE[1],botPoseBLUE[2],new Rotation3d(botPoseBLUE[3],botPoseBLUE[4],botPoseBLUE[5]));    
    }
  }

  public double getTimestamp(){
    return Timer.getFPGATimestamp() - (tl.getDouble(0) + 11) / 1000;
  }

  public AprilTagFieldLayout getFieldLayout(){
    return fieldLayout;
  }
}
