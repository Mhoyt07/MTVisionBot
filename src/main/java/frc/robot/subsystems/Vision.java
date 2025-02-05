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
  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    target_pose = table.getEntry("targetpose_cameraspace");
  }

  public double[] get_target_pose() {
    return target_pose.getDoubleArray(new double[] {0, 0, 0, 0, 0, 0});
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumberArray("target_pose", get_target_pose());
  }
}
