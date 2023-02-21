//fix timer locking drivetrain on shots
//create Speed buttons
//Auto choosher
//multiple trajectories
//pathplanning
//joystick arm control
//reset gyro

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Arm;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Intake m_intake = new Intake();
  private final Launcher m_launcher = new Launcher();
  private final Arm m_arm = new Arm();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
        
        }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    
    // set wheels in x pattern  Acts a a brake
    new JoystickButton(m_driverController, 10)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
//start button on driver resets gyro--laucher facing driver station
    new JoystickButton(m_driverController, 8)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.zeroHeading(),
                m_robotDrive));        
    /* 
                new JoystickButton(m_driverController, 7)
                .whileTrue(new RunCommand(
                    () -> m_robotDrive.changeSpeedHigh(),
                    m_robotDrive)).whileFalse(new RunCommand(()-> m_robotDrive.changeSpeedLow(),
                    m_robotDrive));
                    
       */          
                        

     //turns on intake when presses (left bumper) turns off when released       
    new JoystickButton(m_driverController, 5)
        .onTrue(new RunCommand(
            () -> m_intake.intakeRun(), 
            m_intake)).onFalse(new RunCommand(
                () -> m_intake.intakeStop(),
                m_intake)); 
                
           
    //Reverses intake 
    new JoystickButton(m_driverController, 6)
        .whileTrue(new RunCommand(
                () -> m_intake.intakeReverse(), 
                m_intake)).onFalse(new RunCommand(
                    () -> m_intake.intakeStop(),
                    m_intake)); 
  
    //low shot                changed from whiletrue
    new JoystickButton(m_driverController, 1)
                    .whileTrue(new RunCommand(
                        () -> m_launcher.lowShot(), 
                        m_launcher)); 


          
    //high shot--locks drive in current state needs fixing                    
//changed from whiletrue
    new JoystickButton(m_driverController, 4)
                        .whileTrue(new RunCommand(
                            () -> m_launcher.highShot(), 
                            m_launcher)); 
                            
    //sets arm to up position                        
    new JoystickButton(m_driverController, 3)
                            .whileTrue(new RunCommand(
                                () -> m_arm.armUp(), 
                                m_arm)); 

    //sets arm to down position
    new JoystickButton(m_driverController, 2)
                            .whileTrue(new RunCommand(
                                () -> m_arm.armDown(), 
                                m_arm)); 
     
                                
    new JoystickButton(m_operatorController, 2)
                            .whileTrue(new RunCommand(
                                    () -> m_arm.armJoystickControl(m_operatorController.getLeftY()*-.1), 
                                    m_arm));                             
   
                                //joystick arm control
    //m_arm.armJoystickControl(m_operatorController.getLeftY());   
     
        
                           //change max speed
      //new JoystickButton(m_driverController, 9)
        //.whileTrue(getAutonomousCommand())(DriveConstants.kMaxSpeedMetersPerSecond = 4,m_robotDrive);
    
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);
       
    // An example trajectory to follow. All units in meters.
    Trajectory straigthGamePiece = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 0), new Translation2d(3, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(4.7, 0, new Rotation2d(0)),
        config);

        Trajectory backToStart = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(-1, 0), new Translation2d(-3, 0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(-4.7, 0, new Rotation2d(0)),
            config);






    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        straigthGamePiece,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
        

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);


        
    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(straigthGamePiece.getInitialPose());


     
    
         
    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }
}
