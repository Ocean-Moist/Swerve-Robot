// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain; // never used

public class SwerveModule extends SubsystemBase {
    private static final double WHEEL_RADIUS = 0.0508;
    private static final int ENCODER_RESOLUTION = 4096;

    private static final double MODULE_MAX_ANGULAR_VELOCITY = Drivetrain.MAX_ANGULAR_SPEED;
    private static final double MODULE_MAX_ANGULAR_ACCELERATION = 2 * Math.PI; // radians per second squared

    private final SpeedController driveMotor;
    private final SpeedController turningMotor;

    private final Encoder driveEncoder;
    private final Encoder turningEncoder;

    private final PIDController drivePIDController = new PIDController(1, 0, 0); // needs tuning

    private final ProfiledPIDController turningPIDController
            = new ProfiledPIDController(1, 0, 0,
            new TrapezoidProfile.Constraints(MODULE_MAX_ANGULAR_VELOCITY, MODULE_MAX_ANGULAR_ACCELERATION)); // needs tuning

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1, 3); // example gains (need to tune)
    private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(1, 0.5); // example gains (need to tune)

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
     *
     * @param driveMotorChannel      PWM output for the drive motor.
     * @param turningMotorChannel    PWM output for the turning motor.
     * @param driveEncoderChannelA   DIO input for the drive encoder channel A
     * @param driveEncoderChannelB   DIO input for the drive encoder channel B
     * @param turningEncoderChannelA DIO input for the turning encoder channel A
     * @param turningEncoderChannelB DIO input for the turning encoder channel B
     */
    public SwerveModule(int driveMotorChannel, int turningMotorChannel, int driveEncoderChannelA,
                        int driveEncoderChannelB,
                        int turningEncoderChannelA,
                        int turningEncoderChannelB) {
        driveEncoder = new Encoder(driveEncoderChannelA, driveEncoderChannelB);
        turningEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);
        driveMotor = new PWMVictorSPX(driveMotorChannel);
        turningMotor = new PWMVictorSPX(turningMotorChannel);

        // Set the distance per pulse for the drive encoder. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        driveEncoder.setDistancePerPulse(2 * Math.PI * WHEEL_RADIUS / ENCODER_RESOLUTION);

        // Set the distance (in this case, angle) per pulse for the turning encoder.
        // This is the the angle through an entire rotation (2 * wpi::math::pi)
        // divided by the encoder resolution.
        turningEncoder.setDistancePerPulse(2 * Math.PI / ENCODER_RESOLUTION);

        // Limit the PID Controller's input range between -pi and pi and set the input
        // to be continuous.
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getRate(), new Rotation2d(turningEncoder.get()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state =
                SwerveModuleState.optimize(desiredState, new Rotation2d(turningEncoder.get()));

        // Calculate the drive output from the drive PID controller.
        final double driveOutput =
                drivePIDController.calculate(driveEncoder.getRate(), state.speedMetersPerSecond);

        final double driveFeedforwardOutput = driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput =
                turningPIDController.calculate(turningEncoder.get(), state.angle.getRadians());

        final double turnFeedforwardOutput =
                turnFeedforward.calculate(turningPIDController.getSetpoint().velocity);

        driveMotor.setVoltage(driveOutput + driveFeedforwardOutput);
        turningMotor.setVoltage(turnOutput + turnFeedforwardOutput);
    }

    /**
     * Zeros all the SwerveModule encoders.
     */
    public void resetEncoders() {
        driveEncoder.reset();
        turningEncoder.reset();
    }

}
