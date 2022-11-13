// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;


/** Add your docs here. */
public class SwerveModule {
    TalonFX rotation;
    TalonFX drive;
    public SwerveModule(int driveID, int rotationID) {
        rotation = new TalonFX(rotationID);
        drive = new TalonFX(driveID);
    }
    double ANGLECONSTANT = 2048/360.0;
    double SPEEDCONSTANT = 2533.109;

    public void setSpeed(double speed){
        drive.set(ControlMode.Velocity, speed*SPEEDCONSTANT);
        
    }

    public void setRotation(Rotation2d angle){
        
        rotation.set(ControlMode.Position, angle.getDegrees() * ANGLECONSTANT);
        
        
    }


}
