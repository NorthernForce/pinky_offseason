package frc.robot.robots;

import java.util.Map;
import java.util.Optional;

import org.northernforce.commands.NFRRotatingArmJointWithJoystick;
import org.northernforce.encoders.NFRAbsoluteEncoder;
import org.northernforce.encoders.NFRCANCoder;
import org.northernforce.encoders.NFREncoder;
import org.northernforce.motors.MotorEncoderMismatchException;
import org.northernforce.motors.NFRSparkMax;
import org.northernforce.motors.NFRTalonFX;
import org.northernforce.subsystems.arm.NFRRotatingArmJoint;
import org.northernforce.subsystems.arm.NFRRotatingArmJoint.NFRRotatingArmJointConfiguration;
import org.northernforce.util.NFRRobotContainer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class PinkyContainer implements NFRRobotContainer {
    NFRRotatingArmJoint rotatingJoint;
    NFRRotatingArmJoint wristJoint;
    public PinkyContainer() {
        NFRRotatingArmJointConfiguration rotatingJointConfiguration = new NFRRotatingArmJointConfiguration("rotatingJoint")
            .withUseLimits(false)
            .withUseIntegratedLimits(true)
            .withGearbox(DCMotor.getFalcon500(2))
            .withLimits(Rotation2d.fromDegrees(-95), Rotation2d.fromDegrees(71)); // TODO
        TalonFXConfiguration rotatingJointMotorConfiguration = new TalonFXConfiguration();
        rotatingJointMotorConfiguration.MotionMagic.MotionMagicAcceleration = 0; // TODO
        rotatingJointMotorConfiguration.MotionMagic.MotionMagicCruiseVelocity = 0; // TODO
        rotatingJointMotorConfiguration.Slot0.kP = 0; // TODO
        rotatingJointMotorConfiguration.Slot0.kI = 0; // TODO
        rotatingJointMotorConfiguration.Slot0.kD = 0; // TODO
        rotatingJointMotorConfiguration.Slot0.kV = 0; // TODO
        rotatingJointMotorConfiguration.Slot0.kS = 0; // TODO
        rotatingJointMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rotatingJointMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rotatingJointMotorConfiguration.MotionMagic.MotionMagicCruiseVelocity = 0;
        rotatingJointMotorConfiguration.MotionMagic.MotionMagicAcceleration = 0;
        NFRTalonFX rotatingJointMotor = new NFRTalonFX(rotatingJointMotorConfiguration, 5,6);
        rotatingJointMotor.setFollowerOppose(0);
        NFRCANCoder rotatingJointCANCoder = new NFRCANCoder(13);
        rotatingJointCANCoder.setRange(true);
        rotatingJointCANCoder.setInverted(true);
        try
        {
            rotatingJointMotor.setSelectedEncoder(rotatingJointCANCoder);
        }
        catch (MotorEncoderMismatchException e)
        {
            e.printStackTrace();
        }
        rotatingJoint = new NFRRotatingArmJoint(rotatingJointConfiguration, rotatingJointMotor, Optional.empty());
        Shuffleboard.getTab("General").addDouble("Arm Angle", () -> rotatingJoint.getRotation().getDegrees());
        Shuffleboard.getTab("General").add("Reset CANCoder",
            Commands.runOnce(
                () -> rotatingJointCANCoder.setAbsoluteOffset(rotatingJointCANCoder.getAbsoluteOffset()
                    - rotatingJointCANCoder.getPosition())));
        
        NFRRotatingArmJointConfiguration wristJointConfiguration = new NFRRotatingArmJointConfiguration("wristJoint")
            .withUseLimits(false)
            .withUseIntegratedLimits(true)
            .withLimits(Rotation2d.fromDegrees(-95), Rotation2d.fromDegrees(71)); // TODO
        NFRSparkMax wristJointMotor = new NFRSparkMax(MotorType.kBrushed, 10);
        wristJointMotor.getPIDController().setP(0.0); // TODO
        wristJointMotor.getPIDController().setI(0.0); // TODO
        wristJointMotor.setInverted(false); // TODO
        try
        {
            wristJointMotor.setSelectedEncoder(wristJointMotor.getAbsoluteEncoder().get());
        }
        catch (MotorEncoderMismatchException e)
        {
            e.printStackTrace();
        }
        wristqJoint = new NFRRotatingArmJoint(wristJointConfiguration, wristJointMotor, Optional.empty());
        Shuffleboard.getTab("General").addDouble("Wrist Angle", () -> wristJoint.getRotation().getDegrees());
        Shuffleboard.getTab("General").add("Reset Encoder",
            Commands.runOnce(
                () -> wristJointMotor.getAbsoluteEncoder().get().setAbsoluteOffset(wristJointMotor.getAbsoluteEncoder().get().getAbsoluteOffset()
                    - wristJointMotor.getAbsoluteEncoder().get().getPosition())));
        
    }
    
    @Override
    public void bindOI(GenericHID driverHID, GenericHID manipulatorHID) {
        XboxController manipulatorController = (XboxController)manipulatorHID;
        rotatingJoint.setDefaultCommand(new NFRRotatingArmJointWithJoystick(rotatingJoint,
            () -> -MathUtil.applyDeadband(manipulatorController.getLeftY(), 0.1, 1)));
        wristJoint.setDefaultCommand(new NFRRotatingArmJointWithJoystick(wristJoint,
            () -> -MathUtil.applyDeadband(manipulatorController.getRightY(), 0.1, 1)));
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