package frc.robot.robots;

import java.util.Map;

import org.northernforce.util.NFRRobotContainer;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class PinkyContainer implements NFRRobotContainer {
    public PinkyContainer() {
        // TODO
    }

    @Override
    public void bindOI(GenericHID driverHID, GenericHID manipulatorHID) {
        // TODO
    }

    @Override
    public Map<String, Command> getAutonomousOptions() {
        // TODO
        return Map.of();
    }

    @Override
    public Map<String, Pose2d> getStartingLocations() {
        // TODO
        return Map.of();
    }

    @Override
    public Pair<String, Command> getDefaultAutonomous() {
        // TODO
        return Pair.of("nothing", new InstantCommand());
    }

    @Override
    public void setInitialPose(Pose2d pose) {
        // TODO
    }
}