// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Intake extends SubsystemBase {
  
 
  private final CANSparkMax intaker = new CANSparkMax(IntakeConstants.kIntakeID,MotorType.kBrushless );
  
  
  /** Creates a new Intake. */
  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeRun(){
    intaker.set(IntakeConstants.intakeSpeed);
  }

  public void intakeSet(double speed2){
    intaker.set(speed2);

  }
  public void intakeStop(){
    intaker.set(0);

  }
  public void intakeReverse(){
    intaker.set(-IntakeConstants.intakeSpeed);

}
}