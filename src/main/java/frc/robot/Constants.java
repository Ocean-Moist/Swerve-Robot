package frc.robot;


public final class Constants {
    public static final class DriveConstants {
        public static final int frontLeftDrivePort = 0;
        public static final int frontLeftTurnPort = 0;
        public static final int frontRightDrivePort = 0;
        public static final int frontRightTurnPort = 0;
        public static final int backLeftDrivePort = 0;
        public static final int backLeftTurnPort = 0;
        public static final int backRightDrivePort = 0;
        public static final int backRightTurnPort = 0;

        // if you want you can change the A and B pairs to int[] e.x public static final int[] FrontLeftTurningEncoderPorts = new int[] {0,1};
        public static final int frontLeftTurnEncoderPortA = 0;
        public static final int frontLeftTurnEncoderPortB = 0;
        public static final int frontRightTurnEncoderPortA = 0;
        public static final int frontRightTurnEncoderPortB = 0;
        public static final int backLeftTurnEncoderPortA = 0;
        public static final int backLeftTurnEncoderPortB = 0;
        public static final int backRightTurnEncoderPortA = 0;
        public static final int backRightTurnEncoderPortB = 0;

        public static final int frontLeftDriveEncoderPortA = 0;
        public static final int frontLeftDriveEncoderPortB = 0;
        public static final int frontRightDriveEncoderPortA = 0;
        public static final int frontRightDriveEncoderPortB = 0;
        public static final int backLeftDriveEncoderPortA = 0;
        public static final int backLeftDriveEncoderPortB = 0;
        public static final int backRightDriveEncoderPortA = 0;
        public static final int backRightDriveEncoderPortB = 0;

        public static final double speedScale = 1.0;

        public static final double minimumDrivePower = 0.0;
        public static final double minimumTurnPower = 0.0;
        public static final double maximumDrivePower = 1.0;
        public static final double maximumTurnPower = 1.0;
        public static boolean kGyroReversed = false;
    }

    public static final class AutonomousConstants {

    }

    public static final class OIConstants {
        public static final int xboxControllerPort = 0;
        public static final int fightStickPort = 1;
    }

    public static final class MechanismConstants {

    }

    public static final class PneumaticsConstants {

    }
}
