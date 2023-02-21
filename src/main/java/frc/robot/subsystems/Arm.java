//put PID variables and set them up
//limit switches and hard stops on robot

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

public class Arm extends SubsystemBase {
  
  
  private final TalonFX talonArm = new TalonFX (ArmConstants.kArmID);
  
  
  
  
  
  /** Creates a new Arm. */
  public Arm() {

    talonArm.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,30);
    
   // talonArm.config_kF(0, 0, 0);
    //talonArm.config_kP(0, 0, 0);
    //talonArm.config_kI(0, 0, 0); 
    //talonArm.config_kD(0, 0, 0);
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void armUp(){
talonArm.set(TalonFXControlMode.Position,ArmConstants.kUpPosition);
  }

  public void armDown(){
    talonArm.set(TalonFXControlMode.Position,ArmConstants.kDownPosition);
  }

  public void armJoystickControl(double speed){

    talonArm.set(TalonFXControlMode.PercentOutput,speed);
  }


}
