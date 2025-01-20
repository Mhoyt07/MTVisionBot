// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  PigeonIMU gyro;
  private final SwerveModule[] dt;
  public Drivetrain() {
    gyro = new PigeonIMU(0);
    gyro.configFactoryDefault();
    set_gyro(0);

    this.dt = new SwerveModule[] {
      new SwerveModule(3, 0, Constants.dt.mod3.drive_id, Constants.dt.mod3.turn_id, Constants.dt.mod3.can_coder, Constants.dt.mod3.turn_offset),
      new SwerveModule(0, 1, Constants.dt.mod0.drive_id, Constants.dt.mod0.turn_id, Constants.dt.mod0.can_coder, Constants.dt.mod0.turn_offset),
      new SwerveModule(2, 2, Constants.dt.mod2.drive_id, Constants.dt.mod2.turn_id, Constants.dt.mod2.can_coder, Constants.dt.mod2.turn_offset),
      new SwerveModule(1, 3, Constants.dt.mod1.drive_id, Constants.dt.mod1.turn_id, Constants.dt.mod1.can_coder, Constants.dt.mod1.turn_offset)
    };
  }

  public void set_gyro(double yaw) {
    gyro.setYaw(yaw);
    reset_encoders();
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
      module.set_desired_state(swerve_module_states[module.module_order]);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
