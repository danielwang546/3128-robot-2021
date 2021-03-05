package org.team3128.compbot.commands;

import java.lang.invoke.WrongMethodTypeException;
import java.security.GuardedObject;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import org.team3128.compbot.subsystems.Constants;
import org.team3128.compbot.subsystems.Hopper;
import org.team3128.compbot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import org.team3128.common.utility.Log;

import edu.wpi.first.wpilibj.Timer;

public class CmdIntake implements Command {

    private Set<Subsystem> requirements;

    Hopper hopper;
    Arm arm;
    int startingBallCount;

    public CmdIntake(Hopper hopper, Arm arm) {

        this.requirements = new HashSet<Subsystem>();

        this.hopper = hopper;
        this.requirements.add(hopper);
        this.arm = arm;
        this.requirements.add(arm);

    }

    @Override
    public Set<Subsystem> getRequirements() {
        return requirements;
    }

    @Override
    public void initialize() {
        startingBallCount = hopper.getBallCount();
    }

    @Override
    public void execute() {
        if (arm.ARM_STATE == Arm.ArmState.INTAKE) {
            hopper.setAction(Hopper.ActionState.INTAKING);
        } else {
            arm.setState(Arm.ArmState.INTAKE);
        }
    }

    @Override
    public boolean isFinished() {
        if ((hopper.getBallCount() - startingBallCount) == 1) { // this logic is flawed if there are already three balls
                                                                // in the hopper
                                                                // but, hopefully, this will only be run when the hopper
                                                                // is empty with our autos
            return true;
        }
        return false;
    }
}