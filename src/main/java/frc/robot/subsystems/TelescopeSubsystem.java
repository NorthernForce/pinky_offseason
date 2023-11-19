package frc.robot.subsystems;

import org.northernforce.subsystems.NFRSubsystem;

import edu.wpi.first.wpilibj.Solenoid;

public class TelescopeSubsystem extends NFRSubsystem {
    Solenoid solenoid;

    public TelescopeSubsystem(TelescopeSubsystemConfiguration config, Solenoid solenoid) {
        super(config);
        this.solenoid = solenoid;
    }

    public void extend() {
        solenoid.set(true);
    }

    public void retract() {
        solenoid.set(false);
    }

    public static class TelescopeSubsystemConfiguration extends NFRSubsystemConfiguration {
        public TelescopeSubsystemConfiguration(String name) {
            super(name);
        }
    }
}
