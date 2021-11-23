package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.Drivetrain;


public class Drive extends CommandBase {
    private final Drivetrain m_swerve = new Drivetrain();
    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);
    Drivetrain drivetrain;
    XboxController controller;

    public Drive(Drivetrain drivetrain, XboxController controller) {
        this.drivetrain = drivetrain;
        this.controller = controller;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed =
                -xspeedLimiter.calculate(controller.getX(GenericHID.Hand.kLeft)) // eventually change to MathUtil.applyDeadband(controller.getX(GenericHID.Hand.kLeft), 0.02) (only available in 2022 WPILibs)
                        * Drivetrain.MAX_SPEED;

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final var ySpeed =
                -yspeedLimiter.calculate(controller.getY(GenericHID.Hand.kLeft))  // eventually change to MathUtil.applyDeadband(controller.getY(GenericHID.Hand.kLeft), 0.02) (only available in 2022 WPILibs)
                        * Drivetrain.MAX_SPEED;

        // R stick controls rotation.
        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        final var rot =
                -rotLimiter.calculate(controller.getX(GenericHID.Hand.kRight)) // eventually change to MathUtil.applyDeadband(controller.getX(GenericHID.Hand.kRight), 0.02) (only available in 2022 WPILibs)
                        * Drivetrain.MAX_ANGULAR_SPEED;
        // deadband allows for controller deadzone, makes it less sensitive to noise/nominal values
        drivetrain.drive(xSpeed, ySpeed, rot, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
