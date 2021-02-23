package org.team3128.compbot.autonomous;

import org.team3128.compbot.subsystems.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.team3128.compbot.subsystems.Arm.ArmState;
import org.team3128.compbot.subsystems.Arm;

import java.util.HashSet;
import java.util.Set;

import org.team3128.common.utility.Log;
import edu.wpi.first.wpilibj.Timer;

public class CmdSetArm implements Command {

    private Set<Subsystem> requirements;
    
    Arm arm;
    ArmState armState;
    double timeoutMs, startTime;
    
    public CmdSetArm(Arm arm, ArmState armState, double timeoutMs) {

        this.requirements = new HashSet<Subsystem>();

        this.arm = arm;
        this.requirements.add(arm);

        this.armState = armState;
        this.timeoutMs = timeoutMs;
        
    }
    
    @Override
    public Set<Subsystem> getRequirements() {
        return requirements;
    }

    @Override
    public void initialize() {
        arm.setState(armState);
        startTime = Timer.getFPGATimestamp();
    }
    
    @Override
    public boolean isFinished() {
        if (arm.isReady() || ((Timer.getFPGATimestamp() - startTime) >= timeoutMs)){
            return true;
        } else {
            return false;
        }
    }

}