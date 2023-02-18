// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LauncherConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;



public class Launcher extends SubsystemBase {

 private final DoubleSolenoid highShotSolenoid =
      new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0); 

  private final Solenoid lowShotSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 3);
  private final Solenoid retractSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
      

  /** Creates a new Launcher. */
  public Launcher() {}

  @Override
  public void periodic() {  
    // This method will be called once per scheduler run
    highShotSolenoid.set(DoubleSolenoid.Value.kOff);
    retractSolenoid.set(false);
    lowShotSolenoid.set(false);
  }

  public void lowShot(){
    highShotSolenoid.set(DoubleSolenoid.Value.kOff);
    retractSolenoid.set(false);
    lowShotSolenoid.set(true);
    Timer.delay(.2);
    lowShotSolenoid.set(false);
   retractSolenoid.set(true);
    Timer.delay(0.27);
    retractSolenoid.set(false);


  }

  public void highShot(){
    retractSolenoid.set(false);
    lowShotSolenoid.set(false);
    highShotSolenoid.set(DoubleSolenoid.Value.kForward);
    Timer.delay(.2);
    highShotSolenoid.set(DoubleSolenoid.Value.kReverse);
   retractSolenoid.set(true);
    Timer.delay(0.27);
    retractSolenoid.set(false);







}
}
