package org.team3128.compbot.autonomous;

import org.team3128.compbot.subsystems.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import org.team3128.compbot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.Set;
import java.util.HashSet;


public class CmdSetShooter implements Command {
    
    private Set<Subsystem> requirements;

    Shooter shooter;
    int setpoint;
    
    public CmdSetShooter(Shooter shooter, int setpoint) {
        this.requirements = new HashSet<Subsystem>();
        this.shooter = shooter;
        this.requirements.add(shooter);

        this.setpoint = setpoint;
    }
    
    @Override
    public Set<Subsystem> getRequirements() {
        return requirements;
    }

    @Override
    public void initialize() {
        shooter.setSetpoint(setpoint);
    }
    
    @Override
    public void execute() {

    }
    
    @Override
    public boolean isFinished() {
        if (shooter.isReady()){
            return true;
        } else {
            return false;
        }
    }
}