// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision_Sub extends SubsystemBase {
  /** Creates a new Vision. */
  NetworkTable table;
  NetworkTableEntry target_pose;
  NetworkTableEntry tv;
  double yaw;
  double pitch;
  double roll;
  NetworkTableEntry camera_pose;
  NetworkTableEntry camera_pose_tr;
  ShuffleboardTab vision_tab;
  GenericEntry z;
  public Vision_Sub() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    target_pose = table.getEntry("targetpose_robotspace");
    camera_pose = table.getEntry("camerapose_robotspace_set");
    camera_pose_tr = table.getEntry("camerapose_targetspace");
    tv = table.getEntry("tv");
    vision_tab = Shuffleboard.getTab("Vision");
    set_cam_pose(0, 56, 0);
    z = vision_tab.add("Z offset", get_target_pose()[2]).getEntry();
  }

  public double[] get_target_pose() {
    return target_pose.getDoubleArray(new double[6]);
  }

  public void set_cam_pose(double yaw, double pitch, double roll) {
    camera_pose.setDoubleArray(new double[] {0, 0, 0, yaw, pitch, roll});
  }

  public boolean tag_in_vew() {
    return (tv.getDouble(0) == 0 ? false : true);
  }

  public double[] get_cam_pose() {
    return camera_pose_tr.getDoubleArray(new double[6]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("x", get_cam_pose()[0]);
    SmartDashboard.putNumber("y", get_cam_pose()[1]);
    SmartDashboard.putNumber("z", get_cam_pose()[2]);
    SmartDashboard.putNumber("pitch", get_cam_pose()[3]);
    SmartDashboard.putNumber("yaw", get_cam_pose()[4]);
    SmartDashboard.putNumber("roll", get_cam_pose()[5]);
    set_cam_pose(0, 56, 0);
    SmartDashboard.putNumber("Z offset", get_target_pose()[2]);
  }
}
