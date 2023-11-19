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
        if (config.moduleType == PneumaticsModuleType.CTREPCM) {
            compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
        } else {
            compressor = new Compressor(1, PneumaticsModuleType.REVPH);
        }

    }

    public Solenoid getSolenoid(int channel) {
        return new Solenoid(config.moduleType, channel);
    }

    public static class PneumaticsSubsystemConfiguration extends NFRSubsystemConfiguration
    {
        PneumaticsModuleType moduleType;
        /**
         * Creates a new nfr drive config.
         * @param name a unique subsystem name.
         */
        public PneumaticsSubsystemConfiguration(String name, PneumaticsModuleType type)
        {
            super(name);
            moduleType = type;
        }
    }
}
