package frc.robot.subsystems;

import org.northernforce.subsystems.NFRSubsystem;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class PneumaticsSubsystem extends NFRSubsystem {
    Compressor compressor;
    PneumaticsSubsystemConfiguration config;

    public PneumaticsSubsystem(PneumaticsSubsystemConfiguration config) {
        super(config);
        this.config = config;
        compressor = new Compressor(config.id, config.moduleType);
    }

    public Solenoid getSolenoid(int channel) {
        return new Solenoid(config.id, config.moduleType, channel);
    }

    public static class PneumaticsSubsystemConfiguration extends NFRSubsystemConfiguration
    {
        PneumaticsModuleType moduleType;
        int id;
        /**
         * Creates a new nfr drive config.
         * @param name a unique subsystem name.
         */
        public PneumaticsSubsystemConfiguration(String name, PneumaticsModuleType type, int id)
        {
            super(name);
            this.moduleType = type;
            this.id = id;
        }
    }
}
