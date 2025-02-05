// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  NetworkTable table;
  NetworkTableEntry target_pose;
  double yaw;
  double pitch;
  double roll;
  NetworkTableEntry camera_pose;
  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    target_pose = table.getEntry("targetpose_cameraspace");
    camera_pose = table.getEntry("camerapose_robotspace_set");
    SmartDashboard.putNumber("Yaw", 0);
    SmartDashboard.putNumber("Pitch", 0);

    SmartDashboard.putNumber("Roll", 0);

  }

  public double[] get_target_pose() {
    return target_pose.getDoubleArray(new double[6]);
  }

  public void set_cam_pose(double yaw, double pitch, double roll) {
    camera_pose.setDoubleArray(new double[] {0, 0, 0, yaw, pitch, roll});
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("target_pose x", get_target_pose()[0]);
    SmartDashboard.putNumber("target_pose y", get_target_pose()[1]);
    SmartDashboard.putNumber("target_pose z", get_target_pose()[2]);
    SmartDashboard.putNumber("target_pose yaw", get_target_pose()[3]);
    SmartDashboard.putNumber("target_pose pitch", get_target_pose()[4]);
    SmartDashboard.putNumber("target_pose roll", get_target_pose()[5]);
    SmartDashboard.putBoolean("HI", false);
    yaw = SmartDashboard.getNumber("Yaw", 0);
    pitch = SmartDashboard.getNumber("Pitch", 0);
    roll = SmartDashboard.getNumber("Roll", 0);
    set_cam_pose(yaw, pitch, roll);
  }
}
