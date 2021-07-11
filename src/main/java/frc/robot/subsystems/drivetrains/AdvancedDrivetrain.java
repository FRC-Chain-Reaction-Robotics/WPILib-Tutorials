package frc.robot.subsystems.drivetrains;

import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.kinematics.*;

/**
 * Advanced drivetrain with odometry and kinematics.
 * https://docs.wpilib.org/en/latest/docs/software/pathplanning/trajectory-tutorial/creating-drive-subsystem.html
 * Some things are already defined in Drivetrain.java.
 */
public class AdvancedDrivetrain extends Drivetrain
{
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.69);  //  TODO: tune trackwidth

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds()
    {
        return new DifferentialDriveWheelSpeeds(lfEncoder.getVelocity(), rfEncoder.getVelocity());
    }

    @Override
    public void periodic()
    {
        super.periodic();

        // Update the odometry in the periodic block
        odometry.update(gyro.getRotation2d(), lfEncoder.getPosition(), rfEncoder.getPosition());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts  the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts)
    {
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(-rightVolts);
        dt.feed();
    }

    public DifferentialDriveKinematics getKinematics()
    {
		return kinematics;
    }
    
    public void resetOdometry(Pose2d poseMeters)
    {
        odometry.resetPosition(poseMeters, gyro.getRotation2d());
    }
}
