package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopeSubsystem;

public class Retract extends CommandBase {
    TelescopeSubsystem telescope;
    public Retract(TelescopeSubsystem telescopeSubsystem) {
        this.telescope = telescopeSubsystem;
    }

    @Override
    public void initialize() {
        telescope.retract();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
