// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSub;
 

public class SwerveJoystickCmd extends Command {
  /** Creates a new SwerveJoystickCmd. */
      private final SwerveSub swerveSubsystem;
      private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
      private final Supplier<Boolean> fieldOrientedFunction;
      private final SlewRateLimiter xLimiter, yLimiter, turningLimiter; // slew rate limiter cap the the amount of change of a value
  public SwerveJoystickCmd(
          SwerveSub swerveSubsystem,
          Supplier <Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
          Supplier<Boolean> fieldOrientedFunction) { //
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // gett latest values from joystick

    double xspeed = xSpdFunction.get();
    double yspeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();
    
    //now apply deband,  if joystick doesnt center back to exactly zero, it still stops
    xspeed = Math.abs(xspeed) > OIConstants.kDeadband ? xspeed : 0.0;
    yspeed = Math.abs(yspeed) > OIConstants.kDeadband ? yspeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

    // allows for vioelent joystick movements to be more smooth

    xspeed = xLimiter.calculate(xspeed) *  DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    yspeed = yLimiter.calculate(yspeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond; 
    turningSpeed = turningLimiter.calculate(turningSpeed) *
     DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond ;

    //select orintatin of robot
    ChassisSpeeds chassisSpeeds;
    if(fieldOrientedFunction.get()){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xspeed, yspeed, turningSpeed, swerveSubsystem.getRotation2d());

    }
    else{
      chassisSpeeds = new ChassisSpeeds(xspeed,yspeed, turningSpeed);
    }

    // convert chassis speeds to individual module states; later to switch to velocity
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    // set state to each wheel
    swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
