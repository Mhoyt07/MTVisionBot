// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  NetworkTable table;
  NetworkTableEntry target_pose;
  NetworkTableEntry tv;
  double yaw;
  double pitch;
  double roll;
  NetworkTableEntry camera_pose;
  ShuffleboardTab vision_tab = Shuffleboard.getTab("Vision");
  GenericEntry yaw_val;
  GenericEntry pitch_val;
  GenericEntry roll_val;
  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    target_pose = table.getEntry("targetpose_robotspace");
    camera_pose = table.getEntry("camerapose_robotspace_set");
    tv = table.getEntry("tv");
    set_cam_pose(0, 60, 0);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
