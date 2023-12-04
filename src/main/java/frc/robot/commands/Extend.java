package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopeSubsystem;

public class Extend extends CommandBase {
    TelescopeSubsystem telescope;
    public Extend(TelescopeSubsystem telescopeSubsystem) {
        this.telescope = telescopeSubsystem;
    }

    @Override
    public void initialize() {
        telescope.extend();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
