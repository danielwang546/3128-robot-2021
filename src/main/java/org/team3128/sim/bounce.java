package org.team3128.sim;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team3128.sim.subsystems.DriveSubsystem;

public class bounce extends SequentialCommandGroup {
    public bounce(RobotContainer m_robotContainer, DriveSubsystem mRobotDrive){
         addCommands(
            // new InstantCommand(() -> {
            // mRobotDrive.resetOdometry(m_robotContainer.getTrajectory("Pathweaver/output/Bounce1.wpilib.json").getInitialPose());
            // }),
            m_robotContainer.getAutonomousCommand1()
            // m_robotContainer.getAutonomousCommand("Pathweaver/output/Bounce1.wpilib.json"),
            // m_robotContainer.getAutonomousCommand("Pathweaver/output/Bounce2.wpilib.json"),
            // m_robotContainer.getAutonomousCommand("Pathweaver/output/Bounce3.wpilib.json"),
            // m_robotContainer.getAutonomousCommand("Pathweaver/output/Bounce4.wpilib.json")
        );

        // addCommands(
        //     m_robotContainer.getAutonomousCommand1(),
        //     m_robotContainer.getAutonomousCommand2(),
        //     m_robotContainer.getAutonomousCommand3(),
        //     m_robotContainer.getAutonomousCommand4()
        // );
    }
}
