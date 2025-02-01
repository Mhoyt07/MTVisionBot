// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class dt {
    public static class mod0 {
      public static final int drive_id = 54;
      public static final int turn_id = 58;
      public static final int can_coder = 8;
      public static final Rotation2d turn_offset = Rotation2d.fromDegrees(339.3);
    }
    public static class mod1 {
      public static final int drive_id = 36;
      public static final int turn_id = 52;
      public static final int can_coder = 1;
      public static final Rotation2d turn_offset = Rotation2d.fromDegrees(187.3);
    }
    public static class mod2 {
      public static final int drive_id = 53;
      public static final int turn_id = 61;
      public static final int can_coder = 2;
      public static final Rotation2d turn_offset = Rotation2d.fromDegrees(166.1);
    }
    public static class mod3 {
      public static final int drive_id = 55;
      public static final int turn_id = 59;
      public static final int can_coder = 3;
      public static final Rotation2d turn_offset = Rotation2d.fromDegrees(148.2);
    }

    public static final double drive_motor_ratio = 6.12;
    public static final double turn_motor_ratio = 150 / 7;
    public static final double drive_position_conversion_factor = Math.PI * drive_motor_ratio * 9.5;
    public static final double drive_velocity_conversion_factor = drive_position_conversion_factor / 60;
    public static final double turn_position_conversion_factor = 360 / turn_motor_ratio;

    //lengths in nches to meters
    public static final double robot_length = Units.inchesToMeters(25);
    public static final double robot_width = Units.inchesToMeters(25);

    public static final double turn_kp = 0.00759; //0.00759;
    public static final double turn_ki = 0.00059; //0.00069;
    public static final double turn_kd = 0.0001;  //0.0001;
    public static final double drive_kp = 0.02;
    public static final double drive_ki = 0;
    public static final double drive_kd = 0;

    //max values in meters per second or radians per second
    public static final double max_speed = 5;
    public static final double max_angular_speed = 7.0;

    //swerve drice kinematics
    public static final SwerveDriveKinematics swerve_map = new SwerveDriveKinematics(
      new Translation2d(robot_length / 2, robot_width / 2),
      new Translation2d(robot_length / 2, -robot_width / 2),
      new Translation2d(-robot_length / 2, robot_width / 2),
      new Translation2d(-robot_length / 2, -robot_width / 2)
    );
  }
}
