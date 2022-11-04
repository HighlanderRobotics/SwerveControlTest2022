// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.kinematics.SwerveModuleState;


/** Add your docs here. */
public class SwerveModule {
    TalonFX rotation;
    TalonFX drive;

    

    public void setSpeed(double speed){
        drive.set(ControlMode.Velocity, speed*2533.109);
        
    }

    public void setRotation(double angle){
        rotation.set(ControlMode.Position, angle);
        
        
    }


}
