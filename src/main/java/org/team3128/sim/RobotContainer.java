// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team3128.sim;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import org.team3128.sim.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import java.nio.file.*;
import java.io.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the Robot periodic methods (other than the scheduler
 * calls). Instead, the structure of the robot (including subsystems, commands,
 * and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();

    // The driver's controller
    XboxController m_driverController = new XboxController(Constants.OIConstants.kDriverControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // Set the default drive command to split-stick arcade drive
        m_robotDrive.setDefaultCommand(
                // A split-stick arcade command, with forward/backward controlled by the left
                // hand, and turning controlled by the right.
                new RunCommand(() -> m_robotDrive.arcadeDrive(-m_driverController.getY(GenericHID.Hand.kRight),
                        m_driverController.getX(GenericHID.Hand.kLeft)), m_robotDrive));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * calling passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        // Drive at half speed when the right bumper is held
        new JoystickButton(m_driverController, Button.kBumperRight.value)
                .whenPressed(() -> m_robotDrive.setMaxOutput(0.5)).whenReleased(() -> m_robotDrive.setMaxOutput(1));
    }

    public DriveSubsystem getRobotDrive() {
        return m_robotDrive;
    }

    /** Zeros the outputs of all subsystems. */
    public void zeroAllOutputs() {
        m_robotDrive.tankDriveVolts(0, 0);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */


    public Trajectory getTrajectory(String trajPath) {
        String trajectoryJSON = trajPath;
        Trajectory exampleTrajectory = new Trajectory();

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            System.out.println(ex);
        }

        return exampleTrajectory;
    }

    public Command getAutonomousCommand(String trajPath) {

        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics, 7);
        String trajectoryJSON = trajPath;
        Trajectory exampleTrajectory = new Trajectory();

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            System.out.println(ex);
        }

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, m_robotDrive);
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }






    public Command getAutonomousCommand() {

        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics, 7);

        // Create config for trajectory
        // TrajectoryConfig config = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        //         Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //                 // Add kinematics to ensure max speed is actually obeyed
        //                 .setKinematics(Constants.DriveConstants.kDriveKinematics)
        //                 // Apply the voltage constraint
        //                 .addConstraint(autoVoltageConstraint).setReversed(false);

        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(1.5, 0.7, new Rotation2d(0)),
        //         List.of(new Translation2d(2.5, 1.4), 
        //         new Translation2d(4.7, 2.8), 
        //         new Translation2d(6.6, 1.5),         
        //         new Translation2d(7.7, 0.7), 
        //         new Translation2d(8.5, 1.7), 
        //         new Translation2d(7.6, 2.5),  
        //         new Translation2d(6.6, 1.6), 
        //         new Translation2d(4.7, 0.7), 
        //         new Translation2d(2.6, 1.5)),
        //         new Pose2d(1.3, 2.4, new Rotation2d(3.14)),
        //         config);

        String trajectoryJSON = "output/Bounce2.wpilib.json";
        Trajectory exampleTrajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            System.out.println("here");
            System.out.println(trajectoryPath);
            exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            System.out.println("here");
        } catch (IOException ex) {
            System.out.println(ex);
            // DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON,
            // ex.getStackTrace());
        }

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, m_robotDrive);

        // Reset odometry to starting pose of trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }

    public Command getAutonomousCommandSlalom() {

        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics, 7);

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint).setReversed(false);

        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(1.5, 2.3, new Rotation2d(0)),
        //         List.of(new Translation2d(2.3, 2.9)),
        //         new Pose2d(2.4, 3.8, new Rotation2d(1.57)),
        //         config);

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(1.4, 0.8, new Rotation2d(0)),
                                List.of(
                                new Translation2d(2.5, 1.5),
                                new Translation2d(4.6, 2.7),
                                new Translation2d(6.6, 1.5),
                                new Translation2d(7.5, 0.7),
                                new Translation2d(8.5, 1.6),
                                new Translation2d(7.5, 2.5),
                                new Translation2d(6.6, 1.5),
                                new Translation2d(4.4, 1),
                                new Translation2d(2.5, 1.5)),
                                new Pose2d(1, 2.5, new Rotation2d(3.14)),
                                config);


        // String trajectoryJSON = "Pathweaver/output/Bounce1.wpilib.json";
        // Trajectory exampleTrajectory = new Trajectory();
        // try {
        //     Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        //     System.out.println("here");
        //     System.out.println(trajectoryPath);
        //     exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        //     System.out.println("here");
        // } catch (IOException ex) {
        //     System.out.println(ex);
        //     // DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON,
        //     // ex.getStackTrace());
        // }




        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, m_robotDrive);

        // Reset odometry to starting pose of trajectory.
       // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }


    public Command getAutonomousCommandBarrel() {

        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics, 7);

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint).setReversed(false);

        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(1.5, 2.3, new Rotation2d(0)),
        //         List.of(new Translation2d(2.3, 2.9)),
        //         new Pose2d(2.4, 3.8, new Rotation2d(1.57)),
        //         config);

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(1.4, 2.2, new Rotation2d(0)),
                                List.of(
                                new Translation2d(4, 2),
                                new Translation2d(3.8, 1),
                                new Translation2d(3.3, 1.7),
                                new Translation2d(5.2, 2.2),
                                new Translation2d(6.7, 2.8),
                                new Translation2d(6.3, 3.7),
                                new Translation2d(5.7, 2.2),
                                new Translation2d(7.5, 1),
                                new Translation2d(8.2, 1.7),
                                new Translation2d(7.3, 2.6)),
                                new Pose2d(1, 2.7, new Rotation2d(3.14)),
                                config);


        // String trajectoryJSON = "Pathweaver/output/Bounce1.wpilib.json";
        // Trajectory exampleTrajectory = new Trajectory();
        // try {
        //     Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        //     System.out.println("here");
        //     System.out.println(trajectoryPath);
        //     exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        //     System.out.println("here");
        // } catch (IOException ex) {
        //     System.out.println(ex);
        //     // DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON,
        //     // ex.getStackTrace());
        // }




        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, m_robotDrive);

        // Reset odometry to starting pose of trajectory.
       // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }


    public Command getAutonomousCommandBounce1() {

        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics, 7);

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint).setReversed(false);

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(1.1, 2.3, new Rotation2d(0)),
                                List.of(
                                new Translation2d(2.2, 2.6)),
                                new Pose2d(2.4, 3.5, new Rotation2d(1.57)),
                                config);

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, m_robotDrive);

        // Reset odometry to starting pose of trajectory.
       // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }

    public Command getAutonomousCommandBounce2() {

        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics, 7);

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint).setReversed(true);

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(2.4, 3.5, new Rotation2d(1.57)),
                                List.of(
                                new Translation2d(3.4, 1.1),
                                new Translation2d(4.5, 1.4)),
                                new Pose2d(4.6, 3.6, new Rotation2d(4.71)),
                                config);

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, m_robotDrive);

        // Reset odometry to starting pose of trajectory.
        //m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }

    public Command getAutonomousCommandBounce3() {

        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics, 7);

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint).setReversed(false);

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(4.6, 3.5, new Rotation2d(4.71)),
                                List.of(
                                new Translation2d(4.8, 1.3),
                                new Translation2d(6, 0.9),
                                new Translation2d(6.6, 1.7)),
                                new Pose2d(6.7, 3.5, new Rotation2d(1.57)),
                                config);

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, m_robotDrive);

        // Reset odometry to starting pose of trajectory.
        //m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }


    public Command getAutonomousCommandBounce4() {

        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics, 7);

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint).setReversed(true);

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(6.7, 3.5, new Rotation2d(1.57)),
                                List.of(
                                new Translation2d(7.5, 2.4)),
                                new Pose2d(8.2, 2.4, new Rotation2d(3.14)),
                                config);

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, m_robotDrive);

        // Reset odometry to starting pose of trajectory.
        //m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }


    public Command getAutonomousCommand1() {

        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics, 7);

        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.DriveConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint).setReversed(false);

        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(1.5, 2.3, new Rotation2d(0)),
        //         List.of(new Translation2d(2.3, 2.9)),
        //         new Pose2d(2.4, 3.8, new Rotation2d(1.57)),
        //         config);

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(3, 2)),
                new Pose2d(5, 0, new Rotation2d(0)),
                config);


        // String trajectoryJSON = "Pathweaver/output/Bounce1.wpilib.json";
        // Trajectory exampleTrajectory = new Trajectory();
        // try {
        //     Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        //     System.out.println("here");
        //     System.out.println(trajectoryPath);
        //     exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        //     System.out.println("here");
        // } catch (IOException ex) {
        //     System.out.println(ex);
        //     // DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON,
        //     // ex.getStackTrace());
        // }




        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, m_robotDrive);

        // Reset odometry to starting pose of trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }

    public Command getAutonomousCommand2() {

        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics, 7);

        // Create config for trajectory
        // TrajectoryConfig config = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        //         Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //                 // Add kinematics to ensure max speed is actually obeyed
        //                 .setKinematics(Constants.DriveConstants.kDriveKinematics)
        //                 // Apply the voltage constraint
        //                 .addConstraint(autoVoltageConstraint).setReversed(true);

        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(2.4, 3.8, new Rotation2d(1.57)),
        //         List.of(new Translation2d(2.8, 2), 
        //         new Translation2d(3.8, 0.9)),
        //         new Pose2d(4.6, 3.7, new Rotation2d(4.71)),
        //         config);

        String trajectoryJSON = "Pathweaver/output/Bounce2.wpilib.json";
        Trajectory exampleTrajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            System.out.println("here");
            System.out.println(trajectoryPath);
            exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            System.out.println("here");
        } catch (IOException ex) {
            System.out.println(ex);
            // DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON,
            // ex.getStackTrace());
        }

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, m_robotDrive);

        // Reset odometry to starting pose of trajectory.
        //m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        //Timer.delay(10.0);
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }

    public Command getAutonomousCommand3() {

        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics, 7);

        // Create config for trajectory
        // TrajectoryConfig config = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        //         Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //                 // Add kinematics to ensure max speed is actually obeyed
        //                 .setKinematics(Constants.DriveConstants.kDriveKinematics)
        //                 // Apply the voltage constraint
        //                 .addConstraint(autoVoltageConstraint).setReversed(true);

        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(2.4, 3.8, new Rotation2d(1.57)),
        //         List.of(new Translation2d(2.8, 2), 
        //         new Translation2d(3.8, 0.9)),
        //         new Pose2d(4.6, 3.7, new Rotation2d(4.71)),
        //         config);

        String trajectoryJSON = "Pathweaver/output/Bounce3.wpilib.json";
        Trajectory exampleTrajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            System.out.println("here");
            System.out.println(trajectoryPath);
            exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            System.out.println("here");
        } catch (IOException ex) {
            System.out.println(ex);
            // DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON,
            // ex.getStackTrace());
        }

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, m_robotDrive);

        // Reset odometry to starting pose of trajectory.
        //m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        //Timer.delay(10.0);
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }

    public Command getAutonomousCommand4() {

        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics, 7);

        // Create config for trajectory
        // TrajectoryConfig config = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
        //         Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //                 // Add kinematics to ensure max speed is actually obeyed
        //                 .setKinematics(Constants.DriveConstants.kDriveKinematics)
        //                 // Apply the voltage constraint
        //                 .addConstraint(autoVoltageConstraint).setReversed(true);

        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(2.4, 3.8, new Rotation2d(1.57)),
        //         List.of(new Translation2d(2.8, 2), 
        //         new Translation2d(3.8, 0.9)),
        //         new Pose2d(4.6, 3.7, new Rotation2d(4.71)),
        //         config);

        String trajectoryJSON = "Pathweaver/output/Bounce4.wpilib.json";
        Trajectory exampleTrajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            System.out.println("here");
            System.out.println(trajectoryPath);
            exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            System.out.println("here");
        } catch (IOException ex) {
            System.out.println(ex);
            // DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON,
            // ex.getStackTrace());
        }

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
                        Constants.DriveConstants.kvVoltSecondsPerMeter,
                        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
                Constants.DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, m_robotDrive);

        // Reset odometry to starting pose of trajectory.
        //m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        //Timer.delay(10.0);
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }
    
    


}
