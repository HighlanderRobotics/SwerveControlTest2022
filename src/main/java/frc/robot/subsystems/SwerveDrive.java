// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
  /** Creates a new SwerveDrive. */
  
  Translation2d angle;
  Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  SwerveModule moduleA = new SwerveModule();
  SwerveModule moduleB = new SwerveModule();
  SwerveModule moduleC = new SwerveModule();
  SwerveModule moduleD = new SwerveModule();

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
  m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
);
  public SwerveDrive() {
    
    
  }
  void drive(double xspeed, double yspeed, double rotation){
      ChassisSpeeds speeds = new ChassisSpeeds(xspeed, yspeed, rotation);
      
      
      SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds, angle);
      moduleA.setSpeed(moduleStates[0].speedMetersPerSecond);
      moduleB.setSpeed(moduleStates[1].speedMetersPerSecond);
      moduleC.setSpeed(moduleStates[2].speedMetersPerSecond);
      moduleD.setSpeed(moduleStates[3].speedMetersPerSecond);
      moduleA.setRotation(moduleStates[0].angle);
      moduleB.setRotation(moduleStates[1].angle);
      moduleC.setRotation(moduleStates[2].angle);
      moduleD.setRotation(moduleStates[3].angle);

      
      
      

      

  }


  void driveLeft(double speedInMetersPerSecond) {
    ChassisSpeed cs = new ChassisSpeeds(-Math.abs(speedInMetersPerSecond), 0, 0);
    m_kinematics.toSwerveModuleStates(cs);
  }



  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
