package org.team3128.compbot.autonomous;

import org.team3128.compbot.subsystems.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import org.team3128.compbot.subsystems.Arm.ArmState;
import org.team3128.compbot.subsystems.Hopper;
import org.team3128.common.utility.Log;

import java.util.HashSet;
import java.util.Set;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class CmdSetIndexer implements Command {

    private Set<Subsystem> requirements;

    Hopper hopper;
    double power;

    public CmdSetIndexer(Hopper hopper, double power) {
        this.requirements = new HashSet<Subsystem>();
        
        this.hopper = hopper;
        this.requirements.add(hopper);

        this.power = power;
    }

    @Override
    public Set<Subsystem> getRequirements() {
       return requirements;
    }

    @Override
    public void initialize() {
        hopper.HOPPER_FEEDER_MOTOR.set(ControlMode.PercentOutput, power);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}