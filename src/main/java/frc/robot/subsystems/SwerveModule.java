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
        drive.config_kP(0, 0.2);
        drive.config_kI(0, 0);
        drive.config_kD(0, 0);

        
        rotation.config_kP(0, 0.5);
        rotation.config_kI(0, 0);
        rotation.config_kD(0, 0);
    }
    double ANGLECONSTANT = 2048/360.0 * 12.8;
    double SPEEDCONSTANT = 2533.109 * 8.16;

    public void setSpeed(double speedMeterPerSecond){
        drive.set(ControlMode.Velocity, speedMeterPerSecond*SPEEDCONSTANT);
        System.out.println( speedMeterPerSecond * SPEEDCONSTANT);
    }

    public void setRotation(Rotation2d angle){
        
        rotation.set(ControlMode.Position, angle.getDegrees() * ANGLECONSTANT);
        System.out.println( angle.getDegrees() * ANGLECONSTANT);
        
        
    }


}
 