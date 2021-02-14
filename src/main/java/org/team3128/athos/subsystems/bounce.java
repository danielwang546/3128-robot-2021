package org.team3128.athos.subsystems;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team3128.sim.subsystems.DriveSubsystem;
import org.team3128.common.utility.Log;

public class bounce extends SequentialCommandGroup {
    public bounce(PathFinding m_robotContainer, NEODrive mRobotDrive){
        Log.info("MainAthos","2");
        addCommands(
           
            //m_robotContainer.getAutonomousCommandSlalom(mRobotDrive) 
            
             m_robotContainer.getAutonomousCommandBarrel(mRobotDrive) 
           
            // m_robotContainer.getAutonomousCommandBounce1(mRobotDrive), 
            // m_robotContainer.getAutonomousCommandBounce2(mRobotDrive), 
            // m_robotContainer.getAutonomousCommandBounce3(mRobotDrive), 
            // m_robotContainer.getAutonomousCommandBounce4(mRobotDrive) 
        
        
        
        
        //m_robotContainer.getAutonomousCommand2(mRobotDrive) 
        // m_robotContainer.getAutonomousCommand("Pathweaver/output/Bounce1.wpilib.json", mRobotDrive)
            // m_robotContainer.getAutonomousCommand("Pathweaver/output/Bounce2.wpilib.json", mRobotDrive),
            // m_robotContainer.getAutonomousCommand("Pathweaver/output/Bounce3.wpilib.json", mRobotDrive),
            // m_robotContainer.getAutonomousCommand("Pathweaver/output/Bounce4.wpilib.json", mRobotDrive)
        );

    

        // addCommands(
        //     m_robotContainer.getAutonomousCommand1(),
        //     m_robotContainer.getAutonomousCommand2(),
        //     m_robotContainer.getAutonomousCommand3(),
        //     m_robotContainer.getAutonomousCommand4()
        // );
    }
}
