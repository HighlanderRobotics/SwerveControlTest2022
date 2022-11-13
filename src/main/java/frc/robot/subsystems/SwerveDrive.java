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

  SwerveModule fLModule = new SwerveModule(1, 2);
  SwerveModule fRModule = new SwerveModule(5, 6);
  SwerveModule bLModule = new SwerveModule(3, 4); 
  SwerveModule bRModule = new SwerveModule(7, 8);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
  m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
);
  public SwerveDrive() {
    
    
  }
  void drive(double xspeed, double yspeed, double rotation){
      ChassisSpeeds speeds = new ChassisSpeeds(xspeed, yspeed, rotation);
      
      
      SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds, angle);
      
      fLModule.setSpeed(moduleStates[0].speedMetersPerSecond);
      fRModule.setSpeed(moduleStates[1].speedMetersPerSecond);
      bLModule.setSpeed(moduleStates[2].speedMetersPerSecond);
      bRModule.setSpeed(moduleStates[3].speedMetersPerSecond);
      fLModule.setRotation(moduleStates[0].angle);
      fRModule.setRotation(moduleStates[1].angle);
      bLModule.setRotation(moduleStates[2].angle);
      bRModule.setRotation(moduleStates[3].angle);
      

      
      /*
      moduleA.setSpeed(0);
      moduleB.setSpeed(0);
      moduleC.setSpeed(0);
      moduleD.setSpeed(0);
      moduleA.setRotation(moduleStates[0].angle);
      moduleB.setRotation(moduleStates[1].angle);
      moduleC.setRotation(moduleStates[2].angle);
      moduleD.setRotation(moduleStates[3].angle);
*/

      
      
      

      

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
