package frc.robot.commands;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class DriveDistance extends CommandBase {
    Drive driveTrain;
    int pidSlot;
    double driveDistance;
    double speed;
    double leftStartingDistance;
    double rightStartingDistance;
    boolean closedLoop;

    public DriveDistance(Drive driveTrain, double driveDistance, double speed, int pidSlot) {
        addRequirements(driveTrain);
        this.driveTrain = driveTrain;
        this.driveDistance = driveDistance;
        this.speed = speed;
        this.pidSlot = pidSlot;
        this.closedLoop = true;
    }
    public DriveDistance(Drive driveTrain, double driveDistance, double speed) {
        addRequirements(driveTrain);
        this.driveTrain = driveTrain;
        this.driveDistance = driveDistance;
        this.speed = speed;
        this.pidSlot = -1;
        this.closedLoop = false;
    }
    @Override
    public void initialize() {
        this.leftStartingDistance = driveTrain.getLeftDistance();
        this.rightStartingDistance = driveTrain.getRightDistance();
    }
    @Override
    public void execute() {
        DifferentialDriveWheelSpeeds speeds = new DifferentialDriveWheelSpeeds(speed, speed);
        if (closedLoop) {
            driveTrain.driveClosedLoop(speeds, pidSlot);
        } else {
            driveTrain.driveOpenLoop(speeds);
        }
    }
    @Override
    public void end(boolean interrupted) {
        DifferentialDriveWheelSpeeds speeds = new DifferentialDriveWheelSpeeds(0, 0);
        if (closedLoop) {
            driveTrain.driveClosedLoop(speeds, pidSlot);
        } else {
            driveTrain.driveOpenLoop(speeds);
        }
    }
    @Override
    public boolean isFinished() {
        return (driveTrain.getLeftDistance() >= leftStartingDistance + driveDistance) &&
        driveTrain.getRightDistance() >= rightStartingDistance + driveDistance;
    }
}
