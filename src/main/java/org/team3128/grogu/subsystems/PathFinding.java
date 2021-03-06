package org.team3128.grogu.subsystems;

/**
 * @author Tyler Costello and Autonomous Pod
 */



import static edu.wpi.first.wpilibj.XboxController.Button;
import org.team3128.common.utility.Log;
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
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import java.nio.file.*;
import java.io.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PathFinding {
    
    public PathFinding(){
        
    }




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

    public Command getAutonomousCommand(String trajPath, FalconDrive m_robotDrive) {
        Log.info("MainAthos","3");
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.RamseteConstants.ksVolts,
                        Constants.RamseteConstants.kvVoltSecondsPerMeter,
                        Constants.RamseteConstants.kaVoltSecondsSquaredPerMeter),
                Constants.RamseteConstants.kDriveKinematics, 7);
        String trajectoryJSON = trajPath;
        Trajectory exampleTrajectory = new Trajectory();

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            System.out.println(ex);
        }

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(Constants.RamseteConstants.kRamseteB, Constants.RamseteConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.RamseteConstants.ksVolts,
                        Constants.RamseteConstants.kvVoltSecondsPerMeter,
                        Constants.RamseteConstants.kaVoltSecondsSquaredPerMeter),
                Constants.RamseteConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.RamseteConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.RamseteConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, (Subsystem) m_robotDrive);
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }


    public Command getAutonomousCommandSlalom(FalconDrive m_robotDrive) {
        Log.info("MainAthos","3");
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.RamseteConstants.ksVolts,
                        Constants.RamseteConstants.kvVoltSecondsPerMeter,
                        Constants.RamseteConstants.kaVoltSecondsSquaredPerMeter),
                Constants.RamseteConstants.kDriveKinematics, 7);
        //String trajectoryJSON = trajPath;
        TrajectoryConfig config = new TrajectoryConfig(Constants.RamseteConstants.maxVelocity,
        Constants.RamseteConstants.maxAcceleration)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.RamseteConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint).setReversed(false);
        
                        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0, 0, new Rotation2d(0)),
                                List.of(
                                new Translation2d(1, 0.9),
                                new Translation2d(3, 1.5),
                                new Translation2d(4.5, 1.5),
                                new Translation2d(5.5, 0.7),
                                new Translation2d(6.4, 0.4),
                                new Translation2d(7.3, 1),
                                new Translation2d(6.4, 1.7),
                                new Translation2d(5.5, 0.7),
                                new Translation2d(3.3, 0.2),
                                new Translation2d(1.5, 0.2),
                                new Translation2d(0.5, 0.7)),
                                new Pose2d(-1.5, 1.7, new Rotation2d(3.14)),
                                config);

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(Constants.RamseteConstants.kRamseteB, Constants.RamseteConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.RamseteConstants.ksVolts,
                        Constants.RamseteConstants.kvVoltSecondsPerMeter,
                        Constants.RamseteConstants.kaVoltSecondsSquaredPerMeter),
                Constants.RamseteConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.RamseteConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.RamseteConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, (Subsystem) m_robotDrive);
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }

    public Command getAutonomousCommandBarrel(FalconDrive m_robotDrive) {
        Log.info("MainAthos","3");
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.RamseteConstants.ksVolts,
                        Constants.RamseteConstants.kvVoltSecondsPerMeter,
                        Constants.RamseteConstants.kaVoltSecondsSquaredPerMeter),
                Constants.RamseteConstants.kDriveKinematics, 7);
        //String trajectoryJSON = trajPath;
        TrajectoryConfig config = new TrajectoryConfig(Constants.RamseteConstants.maxVelocity,
        Constants.RamseteConstants.maxAcceleration)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.RamseteConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint).setReversed(false);
        
                        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0, 0, new Rotation2d(0)),
                                List.of(
                                new Translation2d(3.7, -0.2),
                                new Translation2d(2.7, -1.2),
                                new Translation2d(2, -0.5),
                                new Translation2d(3, 0.4),
                                new Translation2d(4.1, 0),
                                new Translation2d(5.8, 0.6),
                                new Translation2d(5, 2),
                                new Translation2d(4.4, 0),
                                new Translation2d(7.7, -1.2),
                                new Translation2d(8.4, -0.5),
                                new Translation2d(6.7, 0.4)),
                                new Pose2d(-1, 0.5, new Rotation2d(3.14)),
                                config);

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(Constants.RamseteConstants.kRamseteB, Constants.RamseteConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.RamseteConstants.ksVolts,
                        Constants.RamseteConstants.kvVoltSecondsPerMeter,
                        Constants.RamseteConstants.kaVoltSecondsSquaredPerMeter),
                Constants.RamseteConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.RamseteConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.RamseteConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, (Subsystem) m_robotDrive);
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }

    public Command getAutonomousCommandBounce1(FalconDrive m_robotDrive) {
        Log.info("MainAthos","3");
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.RamseteConstants.ksVolts,
                        Constants.RamseteConstants.kvVoltSecondsPerMeter,
                        Constants.RamseteConstants.kaVoltSecondsSquaredPerMeter),
                Constants.RamseteConstants.kDriveKinematics, 7);
        //String trajectoryJSON = trajPath;
        TrajectoryConfig config = new TrajectoryConfig(Constants.RamseteConstants.maxVelocity,
        Constants.RamseteConstants.maxAcceleration)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.RamseteConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint).setReversed(false);
        
                        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0*0.0254, 0*0.0254, new Rotation2d(0)),
                                List.of(
                                new Translation2d(40*0.0254, 10*0.0254)),
                                new Pose2d(60*0.0254, 65*0.0254, new Rotation2d(1.57)),
                                config);

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(Constants.RamseteConstants.kRamseteB, Constants.RamseteConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.RamseteConstants.ksVolts,
                        Constants.RamseteConstants.kvVoltSecondsPerMeter,
                        Constants.RamseteConstants.kaVoltSecondsSquaredPerMeter),
                Constants.RamseteConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.RamseteConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.RamseteConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, (Subsystem) m_robotDrive);
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }

    public Command getAutonomousCommandBounce2(FalconDrive m_robotDrive) {
        Log.info("MainAthos","3");
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.RamseteConstants.ksVolts,
                        Constants.RamseteConstants.kvVoltSecondsPerMeter,
                        Constants.RamseteConstants.kaVoltSecondsSquaredPerMeter),
                Constants.RamseteConstants.kDriveKinematics, 7);
        //String trajectoryJSON = trajPath;
        TrajectoryConfig config = new TrajectoryConfig(Constants.RamseteConstants.maxVelocity,
        Constants.RamseteConstants.maxAcceleration)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.RamseteConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint).setReversed(true);
        
                        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(60*0.0254, 65*0.0254, new Rotation2d(1.57)),
                                List.of(
                                new Translation2d(90*0.0254, -30*0.0254),
                                new Translation2d(140*0.0254, -45*0.0254)),
                                new Pose2d(150*0.0254, 95*0.0254, new Rotation2d(4.71)),
                                config);

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(Constants.RamseteConstants.kRamseteB, Constants.RamseteConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.RamseteConstants.ksVolts,
                        Constants.RamseteConstants.kvVoltSecondsPerMeter,
                        Constants.RamseteConstants.kaVoltSecondsSquaredPerMeter),
                Constants.RamseteConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.RamseteConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.RamseteConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, (Subsystem) m_robotDrive);
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }

    public Command getAutonomousCommandBounce3(FalconDrive m_robotDrive) {
        Log.info("MainAthos","3");
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.RamseteConstants.ksVolts,
                        Constants.RamseteConstants.kvVoltSecondsPerMeter,
                        Constants.RamseteConstants.kaVoltSecondsSquaredPerMeter),
                Constants.RamseteConstants.kDriveKinematics, 7);
        //String trajectoryJSON = trajPath;
        TrajectoryConfig config = new TrajectoryConfig(Constants.RamseteConstants.maxVelocity,
        Constants.RamseteConstants.maxAcceleration)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.RamseteConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint).setReversed(false);
        
                        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(150*0.0254, 95*0.0254, new Rotation2d(4.71)),
                                List.of(
                                new Translation2d(150*0.0254, 5*0.0254),
                                new Translation2d(220*0.0254, 5*0.0254)),
                                new Pose2d(240*0.0254, 125*0.0254, new Rotation2d(1.57)),
                                config);

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(Constants.RamseteConstants.kRamseteB, Constants.RamseteConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.RamseteConstants.ksVolts,
                        Constants.RamseteConstants.kvVoltSecondsPerMeter,
                        Constants.RamseteConstants.kaVoltSecondsSquaredPerMeter),
                Constants.RamseteConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.RamseteConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.RamseteConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, (Subsystem) m_robotDrive);
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }

    public Command getAutonomousCommandBounce4(FalconDrive m_robotDrive) {
        Log.info("MainAthos","3");
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.RamseteConstants.ksVolts,
                        Constants.RamseteConstants.kvVoltSecondsPerMeter,
                        Constants.RamseteConstants.kaVoltSecondsSquaredPerMeter),
                Constants.RamseteConstants.kDriveKinematics, 7);
        //String trajectoryJSON = trajPath;
        TrajectoryConfig config = new TrajectoryConfig(Constants.RamseteConstants.maxVelocity,
        Constants.RamseteConstants.maxAcceleration)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.RamseteConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint).setReversed(true);
        
                        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(240*0.0254, 125*0.0254, new Rotation2d(1.57)),
                                List.of(
                                new Translation2d(270*0.0254, 100*0.0254)),
                                new Pose2d(300*0.0254, 100*0.0254, new Rotation2d(3.14)),
                                config);

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(Constants.RamseteConstants.kRamseteB, Constants.RamseteConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.RamseteConstants.ksVolts,
                        Constants.RamseteConstants.kvVoltSecondsPerMeter,
                        Constants.RamseteConstants.kaVoltSecondsSquaredPerMeter),
                Constants.RamseteConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.RamseteConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.RamseteConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, (Subsystem) m_robotDrive);
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }
    
    public Command getAutonomousCommandRedA(FalconDrive m_robotDrive) {
        Log.info("MainAthos","3");
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.RamseteConstants.ksVolts,
                        Constants.RamseteConstants.kvVoltSecondsPerMeter,
                        Constants.RamseteConstants.kaVoltSecondsSquaredPerMeter),
                Constants.RamseteConstants.kDriveKinematics, 7);
        //String trajectoryJSON = trajPath;
        TrajectoryConfig config = new TrajectoryConfig(Constants.RamseteConstants.maxVelocity,
        Constants.RamseteConstants.maxAcceleration)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.RamseteConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint).setReversed(false);
        
                        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0*0.0254, 0*0.0254, new Rotation2d(0)),
                                List.of(
                                new Translation2d(60*0.0254, 30*0.0254),
                                new Translation2d(120*0.0254, 0*0.0254),
                                new Translation2d(150*0.0254, 90*0.0254)                                                            
                                ),
                                new Pose2d(330*0.0254, 30*0.0254, new Rotation2d(0)),
                                config);

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(Constants.RamseteConstants.kRamseteB, Constants.RamseteConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.RamseteConstants.ksVolts,
                        Constants.RamseteConstants.kvVoltSecondsPerMeter,
                        Constants.RamseteConstants.kaVoltSecondsSquaredPerMeter),
                Constants.RamseteConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.RamseteConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.RamseteConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, (Subsystem) m_robotDrive);
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }



    public Command getAutonomousCommandRedB(FalconDrive m_robotDrive) {
        Log.info("MainAthos","3");
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.RamseteConstants.ksVolts,
                        Constants.RamseteConstants.kvVoltSecondsPerMeter,
                        Constants.RamseteConstants.kaVoltSecondsSquaredPerMeter),
                Constants.RamseteConstants.kDriveKinematics, 7);
        //String trajectoryJSON = trajPath;
        TrajectoryConfig config = new TrajectoryConfig(Constants.RamseteConstants.maxVelocity,
        Constants.RamseteConstants.maxAcceleration)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.RamseteConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint).setReversed(false);
        
                        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0*0.0254, 0*0.0254, new Rotation2d(0)),
                                List.of(
                                new Translation2d(60*0.0254, 60*0.0254),
                                new Translation2d(120*0.0254, 0*0.0254),
                                new Translation2d(180*0.0254, 60*0.0254)                                                            
                                ),
                                new Pose2d(330*0.0254, 30*0.0254, new Rotation2d(0)),
                                config);

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(Constants.RamseteConstants.kRamseteB, Constants.RamseteConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.RamseteConstants.ksVolts,
                        Constants.RamseteConstants.kvVoltSecondsPerMeter,
                        Constants.RamseteConstants.kaVoltSecondsSquaredPerMeter),
                Constants.RamseteConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.RamseteConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.RamseteConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, (Subsystem) m_robotDrive);
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }

    public Command getAutonomousCommandBlueA(FalconDrive m_robotDrive) {
        Log.info("MainAthos","3");
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.RamseteConstants.ksVolts,
                        Constants.RamseteConstants.kvVoltSecondsPerMeter,
                        Constants.RamseteConstants.kaVoltSecondsSquaredPerMeter),
                Constants.RamseteConstants.kDriveKinematics, 7);
        //String trajectoryJSON = trajPath;
        TrajectoryConfig config = new TrajectoryConfig(Constants.RamseteConstants.maxVelocity,
        Constants.RamseteConstants.maxAcceleration)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.RamseteConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint).setReversed(false);
        
                        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0*0.0254, 0*0.0254, new Rotation2d(0)),
                                List.of(
                                new Translation2d(150*0.0254, -30*0.0254),
                                new Translation2d(180*0.0254, 60*0.0254),
                                new Translation2d(240*0.0254, 30*0.0254)                                                            
                                ),
                                new Pose2d(330*0.0254, 30*0.0254, new Rotation2d(0)),
                                config);

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(Constants.RamseteConstants.kRamseteB, Constants.RamseteConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.RamseteConstants.ksVolts,
                        Constants.RamseteConstants.kvVoltSecondsPerMeter,
                        Constants.RamseteConstants.kaVoltSecondsSquaredPerMeter),
                Constants.RamseteConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.RamseteConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.RamseteConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, (Subsystem) m_robotDrive);
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }

    

    public Command getAutonomousCommandBlueB(FalconDrive m_robotDrive) {
        Log.info("MainAthos","3");
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.RamseteConstants.ksVolts,
                        Constants.RamseteConstants.kvVoltSecondsPerMeter,
                        Constants.RamseteConstants.kaVoltSecondsSquaredPerMeter),
                Constants.RamseteConstants.kDriveKinematics, 7);
        //String trajectoryJSON = trajPath;
        TrajectoryConfig config = new TrajectoryConfig(Constants.RamseteConstants.maxVelocity,
        Constants.RamseteConstants.maxAcceleration)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.RamseteConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint).setReversed(false);
        
                        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0*0.0254, 0*0.0254, new Rotation2d(0)),
                                List.of(
                                new Translation2d(160*0.0254, 0*0.0254),
                                new Translation2d(210*0.0254, 60*0.0254),
                                new Translation2d(270*0.0254, 0*0.0254)                                                            
                                ),
                                new Pose2d(330*0.0254, 30*0.0254, new Rotation2d(0)),
                                config);

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(Constants.RamseteConstants.kRamseteB, Constants.RamseteConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.RamseteConstants.ksVolts,
                        Constants.RamseteConstants.kvVoltSecondsPerMeter,
                        Constants.RamseteConstants.kaVoltSecondsSquaredPerMeter),
                Constants.RamseteConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.RamseteConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.RamseteConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, (Subsystem) m_robotDrive);
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }

























    
    public Command getAutonomousCommand1(FalconDrive m_robotDrive) {
        Log.info("MainAthos","3");
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.RamseteConstants.ksVolts,
                        Constants.RamseteConstants.kvVoltSecondsPerMeter,
                        Constants.RamseteConstants.kaVoltSecondsSquaredPerMeter),
                Constants.RamseteConstants.kDriveKinematics, 7);
        //String trajectoryJSON = trajPath;
        TrajectoryConfig config = new TrajectoryConfig(Constants.RamseteConstants.maxVelocity,
        Constants.RamseteConstants.maxAcceleration)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.RamseteConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint).setReversed(false);
        
                        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0, 0, new Rotation2d(0)),
                                List.of(new Translation2d(3, 3), new Translation2d(7, 0), new Translation2d(3, -3)),
                                new Pose2d(-1, -0.5, new Rotation2d(3.14)),
                                config);

        // try {
        //     Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        //     exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        // } catch (IOException ex) {
        //     System.out.println(ex);
        // }

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(Constants.RamseteConstants.kRamseteB, Constants.RamseteConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.RamseteConstants.ksVolts,
                        Constants.RamseteConstants.kvVoltSecondsPerMeter,
                        Constants.RamseteConstants.kaVoltSecondsSquaredPerMeter),
                Constants.RamseteConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.RamseteConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.RamseteConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, (Subsystem) m_robotDrive);
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }

    public Command getAutonomousCommand2(FalconDrive m_robotDrive) {
        Log.info("MainAthos","3");
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.RamseteConstants.ksVolts,
                        Constants.RamseteConstants.kvVoltSecondsPerMeter,
                        Constants.RamseteConstants.kaVoltSecondsSquaredPerMeter),
                Constants.RamseteConstants.kDriveKinematics, 7);
        //String trajectoryJSON = trajPath;
        TrajectoryConfig config = new TrajectoryConfig(Constants.RamseteConstants.maxVelocity,
        Constants.RamseteConstants.maxAcceleration)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Constants.RamseteConstants.kDriveKinematics)
                        // Apply the voltage constraint
                        .addConstraint(autoVoltageConstraint).setReversed(false);
        
                        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0, 0, new Rotation2d(0)),
                                List.of(
                                new Translation2d(1.424, 1.62),
                                new Translation2d(3.81, 1.286),
                                new Translation2d(6.096, 0.762),
                                new Translation2d(6.858, 0),
                                new Translation2d(7.62, 0.762),
                                new Translation2d(6.858, 1.524),
                                new Translation2d(6.096, 0.762),
                                new Translation2d(3.81, 0),
                                new Translation2d(1.524, 0.762)),
                                new Pose2d(-2, 1, new Rotation2d(3.14)),
                                config);

        // try {
        //     Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        //     exampleTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        // } catch (IOException ex) {
        //     System.out.println(ex);
        // }

        RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
                new RamseteController(Constants.RamseteConstants.kRamseteB, Constants.RamseteConstants.kRamseteZeta),
                new SimpleMotorFeedforward(Constants.RamseteConstants.ksVolts,
                        Constants.RamseteConstants.kvVoltSecondsPerMeter,
                        Constants.RamseteConstants.kaVoltSecondsSquaredPerMeter),
                Constants.RamseteConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
                new PIDController(Constants.RamseteConstants.kPDriveVel, 0, 0),
                new PIDController(Constants.RamseteConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts, (Subsystem) m_robotDrive);
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }



}

