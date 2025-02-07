// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  WPI_PigeonIMU gyro;
  private final SwerveModule[] dt;
  SwerveDrivePoseEstimator pose_estimator;
  Field2d field;
  boolean reject_update;
  public Drivetrain() {
    gyro = new WPI_PigeonIMU(10);
    gyro.configFactoryDefault();
    set_gyro(0);

    this.dt = new SwerveModule[] {
      new SwerveModule(3, 0, Constants.dt.mod3.drive_id, Constants.dt.mod3.turn_id, Constants.dt.mod3.can_coder, Constants.dt.mod3.turn_offset),
      new SwerveModule(0, 1, Constants.dt.mod0.drive_id, Constants.dt.mod0.turn_id, Constants.dt.mod0.can_coder, Constants.dt.mod0.turn_offset),
      new SwerveModule(2, 2, Constants.dt.mod2.drive_id, Constants.dt.mod2.turn_id, Constants.dt.mod2.can_coder, Constants.dt.mod2.turn_offset),
      new SwerveModule(1, 3, Constants.dt.mod1.drive_id, Constants.dt.mod1.turn_id, Constants.dt.mod1.can_coder, Constants.dt.mod1.turn_offset)
    };
    reset_encoders();

    //pose estimator to estimate where the robot is on the field using encoder and yaw data
    pose_estimator = new SwerveDrivePoseEstimator(Constants.dt.swerve_map, get_yaw(), 
      new SwerveModulePosition[] {this.dt[0].get_position(), this.dt[1].get_position(), this.dt[2].get_position(), this.dt[3].get_position()}, new Pose2d());

    //field to visualize where pose estimator thinks robot is
    field = new Field2d();
  }

  public void set_gyro(double yaw) {
    gyro.setYaw(yaw);
  }

  //resets the encoders of each module to the cancoder offset
  public void reset_encoders() {
    for (SwerveModule module : this.dt) {
      module.reset_encoder();
    }
  }

  //returns the yaw of robot
  public Rotation2d get_yaw() {
    return Rotation2d.fromDegrees(gyro.getYaw());
  }

  public double get_yaw_rate() {
    return gyro.getRate();
  }

  //drive funciton
  public void drive(Translation2d translation, double rotation, boolean is_field_relative) {
    //creates swerve module states form joystick values which are in translation and rotation
    SwerveModuleState[] swerve_module_states =
      Constants.dt.swerve_map.toSwerveModuleStates(is_field_relative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
          translation.getX(), translation.getY(), rotation, get_yaw())
        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    //reduces all the swervemodule state speeds if one of them is over the max speed to make it so that none are over but all are still the same ratio
    SwerveDriveKinematics.desaturateWheelSpeeds(swerve_module_states, Constants.dt.max_speed);

    //sets the module states
    for (SwerveModule module: this.dt) {
      module.set_desired_state(swerve_module_states[module.module_number]);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //the order of the modules is front left front right back left then back right because that is the order the swerve map kinematics is defined in
    pose_estimator.update(get_yaw(), new SwerveModulePosition[] {this.dt[0].get_position(), this.dt[1].get_position(), this.dt[2].get_position(), this.dt[3].get_position()});
    //puts the robot position on the robot field and then puts the field on smartdashboard
    LimelightHelpers.SetRobotOrientation("limelight", pose_estimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if (get_yaw_rate() > 720) {
      reject_update = true;
    } else if (mt2.tagCount == 0) {
      reject_update = true;
    } else {
      reject_update = false;
    } if (reject_update == false) {
      pose_estimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      pose_estimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
    }

    field.setRobotPose(pose_estimator.getEstimatedPosition());
    SmartDashboard.putData("Field", field);
    SmartDashboard.putNumber("Gyro Yaw", get_yaw().getDegrees());
  }
}
