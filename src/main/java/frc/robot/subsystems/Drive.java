package frc.robot.subsystems;

import org.northernforce.gyros.NFRGyro;
import org.northernforce.motors.NFRMotorController;
import org.northernforce.subsystems.drive.NFRTankDrive;

public class Drive extends NFRTankDrive {
    public Drive(NFRTankDriveConfiguration config, NFRMotorController leftSide, NFRMotorController rightSide,
            NFRGyro gyro) {
        super(config, leftSide, rightSide, gyro);
    }
    public double getLeftDistance() {
        return leftSide.getSelectedEncoder().getPosition();
    }
    public double getRightDistance() {
        return rightSide.getSelectedEncoder().getPosition();
    }
}
