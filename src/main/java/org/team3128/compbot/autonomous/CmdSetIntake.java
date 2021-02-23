package org.team3128.compbot.autonomous;

import org.team3128.compbot.subsystems.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import org.team3128.compbot.subsystems.Arm.ArmState;
import org.team3128.compbot.subsystems.Hopper;

import java.util.Set;
import java.util.HashSet;

import org.team3128.common.utility.Log;

public class CmdSetIntake implements Command {

    private Set<Subsystem> requirements;

    Hopper hopper;
    double power;

    public CmdSetIntake(Hopper hopper, double power) {
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
        hopper.INTAKE_MOTOR.set(power);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}