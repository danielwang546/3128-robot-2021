package org.team3128.sim;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team3128.sim.subsystems.DriveSubsystem;

public class bounce extends SequentialCommandGroup {
    public bounce(RobotContainer m_robotContainer){
        addCommands(
            m_robotContainer.getAutonomousCommand1(),
            m_robotContainer.getAutonomousCommand2(),
            m_robotContainer.getAutonomousCommand3(),
            m_robotContainer.getAutonomousCommand4()
        );
    }
}
