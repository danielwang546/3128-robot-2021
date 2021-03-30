package org.team3128.grogu.subsystems;

/**
 * @author Tyler Costello and Autonomous Pod
 */



import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.team3128.sim.subsystems.DriveSubsystem;
import org.team3128.common.utility.Log;

public class PathRunner extends SequentialCommandGroup {
    public PathRunner(PathFinding m_robotContainer, FalconDrive mRobotDrive){
        Log.info("MainAthos","2");
        addCommands(
           
           
           
            //m_robotContainer.getAutonomousCommandCompSlalom(mRobotDrive) 

            m_robotContainer.getAutonomousCommandCompBarrel(mRobotDrive) 

            // m_robotContainer.getAutonomousCommandCompBounce1(mRobotDrive),
            // m_robotContainer.getAutonomousCommandCompBounce2(mRobotDrive),
            // m_robotContainer.getAutonomousCommandCompBounce3(mRobotDrive),
            // m_robotContainer.getAutonomousCommandCompBounce4(mRobotDrive)


            //m_robotContainer.getAutonomousCommandCompGalacticSearch(mRobotDrive) 




             //m_robotContainer.getAutonomousCommandSlalom(mRobotDrive) 
         //   m_robotContainer.getAutonomousCommandBarrel(mRobotDrive) 
           
            // m_robotContainer.getAutonomousCommandBounce1(mRobotDrive), 
            // m_robotContainer.getAutonomousCommandBounce2(mRobotDrive), 
            // m_robotContainer.getAutonomousCommandBounce3(mRobotDrive), 
            // m_robotContainer.getAutonomousCommandBounce4(mRobotDrive) 
            // m_robotContainer.getAutonomousCommandRedA(mRobotDrive),
            // m_robotContainer.getAutonomousCommandRedB(mRobotDrive) 
            // m_robotContainer.getAutonomousCommandBlueA(mRobotDrive) 
            //m_robotContainer.getAutonomousCommandBlueB(mRobotDrive) 
          // m_robotContainer.getAutonomousCommandSquare(mRobotDrive) 
        
        
        
        
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

