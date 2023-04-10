/**/
//fix timer locking drivetrain on shots
//Auto choosher
//multiple trajectories
//pathplanning


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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
//import java.util.withTimeouts;

import java.util.HashMap;
//import java.util.io.File;
import java.util.ArrayList;
import java.util.List;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;


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

  // The driver's controller and operator controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);
    
  //Chooser Set up
  SendableChooser<Command> chooser = new SendableChooser<>();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    // Configure the button bindings
    configureButtonBindings();

    // chooser stuff
    chooser.addOption("InsideBlue-OutsideRed", getAutonomousCommand());
    chooser.addOption("Middle", getAutonomousCommand2());
    chooser.addOption("OutsideBlue-InsideRed", getAutonomousCommand3());
    chooser.addOption("Shoot Only", getAutonomousCommand4());
    chooser.addOption("Blue Shoot and Slide Inside", getAutonomousCommand5());
    chooser.addOption("Red Shoot and Slide Inside", getAutonomousCommand6());
    SmartDashboard.putData(chooser);


    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        //squares value of joystick
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(Math.abs(m_driverController.getLeftY())*m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(Math.abs(m_driverController.getLeftX())*m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(Math.abs(m_driverController.getRightX())*m_driverController.getRightX(), OIConstants.kDriveDeadband),
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
    new JoystickButton(m_driverController, 2)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    // set wheels at 90 degrees pattern  Acts a a brake on charge station
    new JoystickButton(m_driverController, 3)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.set90(),
            m_robotDrive));



//start button on driver resets gyro--laucher facing driver station
    new JoystickButton(m_driverController, 8)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.zeroHeading(),
                m_robotDrive));        
    
//changes max speed to low
    new JoystickButton(m_driverController, 5)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.changeSpeedLow(),
                m_robotDrive));


//changes max speed to high
    new JoystickButton(m_driverController, 6)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.changeSpeedHigh(),
                m_robotDrive)); 
              
                        

     //turns on intake when presses (left bumper) turns off when released       
    new JoystickButton(m_operatorController, 5)
        .onTrue(new RunCommand(
            () -> m_intake.intakeRun(), 
            m_intake)).onFalse(new RunCommand(
                () -> m_intake.intakeStop(),
                m_intake)); 
                
           
    //Reverses intake 
    new JoystickButton(m_operatorController, 6)
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
    new JoystickButton(m_operatorController, 4)
                            .whileTrue(new RunCommand(
                                () -> m_arm.armUp(), 
                                m_arm)); 

    //sets arm to down position
    new JoystickButton(m_operatorController, 1)
                            .whileTrue(new RunCommand(
                                () -> m_arm.armDown(), 
                                m_arm)); 
     
    //Joystick arm control  Red button and left stick                            
    new JoystickButton(m_operatorController, 2)
                            .whileTrue(new RunCommand(
                                    () -> m_arm.armJoystickControl(m_operatorController.getLeftY()*-.3), 
                                    m_arm));                             
                          
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        2,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);
       
    // An example trajectory to follow. All units in meters.
    Trajectory straigthGamePiece = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, .1), new Translation2d(3, .1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(4.75, .1, new Rotation2d(0)),
        config);


    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

   
        SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
            straigthGamePiece,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            
    
            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);


Command highLaunch = new RunCommand(
    () -> m_launcher.highShot(), 
    m_launcher);
        
    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(straigthGamePiece.getInitialPose());


     
    
         
    // Run path following command, then stop at the end.
    return highLaunch.withTimeout(.3)
        // .andThen(swerveControllerCommand2)
        .andThen(m_robotDrive.run(() -> m_robotDrive.drive(0.2, 0, 0, false, false))).withTimeout(5.0)
        .andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }


  public Command getAutonomousCommand2() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        1,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);
       
    // An example trajectory to follow. All units in meters.
    Trajectory straigthRamp = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 0), new Translation2d(1.5, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(2.7, 0, new Rotation2d(0)),
        config);


    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

   
        SwerveControllerCommand swerveControllerCommand3 = new SwerveControllerCommand(
            straigthRamp,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            
    
            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);


Command highLaunch = new RunCommand(
    () -> m_launcher.highShot(), 
    m_launcher);
        
    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(straigthRamp.getInitialPose());


     
    
         
    // Run path following command, then stop at the end.
    return highLaunch.withTimeout(.3).
    andThen(swerveControllerCommand3).
    andThen(() -> m_robotDrive.drive(0, 0, 0, false, false)).
    andThen(()->m_robotDrive.set90Auto());
  }



  public Command getAutonomousCommand3() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        2,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);
       
    // An example trajectory to follow. All units in meters.
    Trajectory straigthGamePiece2 = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, -.07), new Translation2d(3, -.07)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(4.75, -.07, new Rotation2d(0)),
        config);


    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

   
        SwerveControllerCommand swerveControllerCommand3 = new SwerveControllerCommand(
            straigthGamePiece2,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            
    
            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);


Command highLaunch = new RunCommand(
    () -> m_launcher.lowShot(), 
    m_launcher);
        
    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(straigthGamePiece2.getInitialPose());


     
    
         
    // Run path following command, then stop at the end.
    return highLaunch.withTimeout(.3)
        //.andThen(swerveControllerCommand3)
        .andThen(m_robotDrive.run(() -> m_robotDrive.drive(0.2, 0, 0, false, false))).withTimeout(5.0)
        .andThen(m_robotDrive.run(() -> m_robotDrive.drive(0, 0, 0, false, false)));
  }

  public Command getAutonomousCommand4() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        2,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);
       
    // An example trajectory to follow. All units in meters.
    Trajectory straigthGamePiece2 = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, -.5), new Translation2d(3, -.5)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(4.75, -.31, new Rotation2d(0)),
        config);


    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

   
        SwerveControllerCommand swerveControllerCommand3 = new SwerveControllerCommand(
            straigthGamePiece2,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            
    
            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);


Command highLaunch = new RunCommand(
    () -> m_launcher.highShot(), 
    m_launcher);
        
    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(straigthGamePiece2.getInitialPose());


     
    
         
    // Run path following command, then stop at the end.
    return highLaunch.withTimeout(.3);
  }
  public Command getAutonomousCommand5() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        2,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);
       
    // An example trajectory to follow. All units in meters.
    Trajectory BlueSlide = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(.8, 1.8), new Translation2d(2.75, 1.8)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(4.75, 4, new Rotation2d(0)),
        config);


    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

   
        SwerveControllerCommand swerveControllerCommand5 = new SwerveControllerCommand(
            BlueSlide,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            
    
            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);


Command highLaunch = new RunCommand(
    () -> m_launcher.highShot(), 
    m_launcher);
        
    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(BlueSlide.getInitialPose());


     
    
         
    // Run path following command, then stop at the end.
    return highLaunch.withTimeout(.3).andThen(swerveControllerCommand5).andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }


  public Command getAutonomousCommand6() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        2,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);
       
    // An example trajectory to follow. All units in meters.
    Trajectory RedSlide = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(.8, -1.8), new Translation2d(2.75, -1.8)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(4.75, -4, new Rotation2d(0)),
        config);


    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

   
        SwerveControllerCommand swerveControllerCommand5 = new SwerveControllerCommand(
            RedSlide,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,
            
    
            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);


Command highLaunch = new RunCommand(
    () -> m_launcher.highShot(), 
    m_launcher);
        
    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(RedSlide.getInitialPose());


     
    
         
    // Run path following command, then stop at the end.
    return highLaunch.withTimeout(.3).andThen(swerveControllerCommand5).andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }

}
