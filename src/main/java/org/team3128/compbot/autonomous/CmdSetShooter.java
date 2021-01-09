package org.team3128.compbot.autonomous;

import org.team3128.compbot.subsystems.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import org.team3128.compbot.subsystems.Shooter;

public class CmdSetShooter implements Command {
    
    Shooter shooter;
    int setpoint;
    
    public CmdSetShooter(Shooter shooter, int setpoint) {
        this.shooter = shooter;
        this.setpoint = setpoint;
        
        addRequirement(shooter);
    }
    
    @Override
    protected void initialize() {
        shooter.setSetpoint(setpoint);
    }
    
    @Override
    protected void execute() {

    }
    
    @Override
    protected boolean isFinished() {
        if (shooter.isReady()){
            return true;
        } else {
            return false;
        }
    }
}