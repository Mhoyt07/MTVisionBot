// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drive extends Command {
  /** Creates a new Drive. */
  Drivetrain dt;
  Joystick joystick_l;
  Joystick joystick_r;
  Translation2d translation;
  double rotation;
  double x;
  double y;
  SlewRateLimiter x_limiter = new SlewRateLimiter(3);
  SlewRateLimiter y_limiter = new SlewRateLimiter(3);
  SlewRateLimiter rotation_limiter = new SlewRateLimiter(3);

  public Drive(Drivetrain dt_imp, Joystick joystick_l, Joystick joystick_r) {
    this.dt = dt_imp;
    this.joystick_l = joystick_l;
    this.joystick_r = joystick_r;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //slew rate limiter slows how fast the values change; it takes 1/3 of a second to go from 0 to 1
    //the deadband makes it so if joystick is between -0.1 and 0.1 it will just return 0
    x = x_limiter.calculate(MathUtil.applyDeadband(this.joystick_l.getX(), 0.1));
    y = -y_limiter.calculate(MathUtil.applyDeadband(this.joystick_l.getY(), 0.1));
    //turns the rotation from a magnitude of 0 to 1 to be in correct speed range using the multiplication
    rotation = rotation_limiter.calculate(MathUtil.applyDeadband(this.joystick_r.getX(), 0.1)) * Constants.dt.max_angular_speed;
    //gets robot translation and rotation from joysticks
    //.times multiplies the translation which has a magnitude between 0 and 1 inclusive by the max speed of the robot
    translation = new Translation2d(x, y).times(Constants.dt.max_speed); 

    //sets the module speeds and positions using the joystick values
    this.dt.drive(translation, rotation, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
