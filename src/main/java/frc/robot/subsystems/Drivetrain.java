package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase
{
	CANSparkMax lf = new CANSparkMax(Constants.LF_MOTOR_ID, MotorType.kBrushless);
	CANSparkMax rf = new CANSparkMax(Constants.RF_MOTOR_ID, MotorType.kBrushless);
	CANSparkMax lb = new CANSparkMax(Constants.LB_MOTOR_ID, MotorType.kBrushless);
	CANSparkMax rb = new CANSparkMax(Constants.RB_MOTOR_ID, MotorType.kBrushless);

	CANEncoder lfEncoder = lf.getEncoder();
	CANEncoder rfEncoder = rf.getEncoder();
	
	Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
	
	DifferentialDrive dt = new DifferentialDrive
	(
		new SpeedControllerGroup(lf, lb),
		new SpeedControllerGroup(rf, rb)
	);

	public static final double AUTON_MAX_OUTPUT = 1.0;	//	TODO: tune maxoutputs
	public static final double TELEOP_MAX_OUTPUT = 1.0;

	public Drivetrain()
	{
		dt.setMaxOutput(AUTON_MAX_OUTPUT);

		resetSensors();

		
		//	The native unit of "encoder ticks" means nothing to us, we want the encoders to report distance traveled in meters.
		//	This value will depend on your gear train, wheel circumference, and drivetrain efficiency.
		
		//	To calculate it, run this process with the value set to 1.0:
		//	1. Drive forwards 1 meter manually, or push the robot.
		//	2. See what distance is printed to SmartDashboard when you move.
		//	3. Set the value to 1 / (whatever distance was printed, should be a number in range of 10-50 or so)
		//	4. Repeat steps 1 and 2 and check if the distance traveled is the same as the distance measured.
		final double positionConversionFactor = 1.0;
		lfEncoder.setPositionConversionFactor(positionConversionFactor);	//	TODO: tune
		lfEncoder.setPositionConversionFactor(positionConversionFactor);


		rf.setInverted(true);	//	change as necessary
		rb.setInverted(true);

		lb.setIdleMode(IdleMode.kCoast);	//	change as necessary. Will control the behavior of the robot when input = 0 from joysticks/auton...
		rf.setIdleMode(IdleMode.kCoast);	//	kCoast will let the wheels glide freely, kBrake will force them to stop.
		rb.setIdleMode(IdleMode.kCoast);
		lf.setIdleMode(IdleMode.kCoast);

		//	save configurations to flash storage on motor controllers
		lf.burnFlash();
		lb.burnFlash();
		rf.burnFlash();
		rb.burnFlash();

		//	sets up gyro for printing angle data as a gyro widget
		SmartDashboard.putData((Sendable) gyro);
	}

	/**
	 * Drives the robot.
	 * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
	 * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.
	 */
	public void drive(double xSpeed, double zRotation)
	{
		dt.arcadeDrive(xSpeed, zRotation);
	}

	/**
	 * Sets the maximum output power of the drivetrain.
	 * @param maxOutput maximum output power from 0.0 to 1.0
	 */
	public void setMaxOutput(double maxOutput)
	{
		dt.setMaxOutput(maxOutput);
	}

	/**
	 * Gets the distance traveled from the encoders.
	 * @return distance traveled in meters
	 */
	public double getDistance()
	{
		// Averages both front encoders... may need to just use one of the two instead if this is buggy
		return (lfEncoder.getPosition() - rfEncoder.getPosition()) / 2;
	}

	/**
	 * Gets the angle that the robot is facing in relative to the last reset / start angle.
	 * @return heading of the robot in degrees
	 */
	public double getAngle()
	{
		return gyro.getAngle();
	}

	public void resetSensors()
	{
		resetEncoders();
		resetGyro();
	}

	public void resetEncoders()
	{
		lfEncoder.setPosition(0);
		rfEncoder.setPosition(0);
	}

	public void resetGyro()
	{
		gyro.reset();
		gyro.calibrate();
	}

	@Override
	public void periodic()
	{
		// prints to SmartDashboard, periodically updating
		SmartDashboard.putNumber("lf position", lfEncoder.getPosition());
		SmartDashboard.putNumber("rf position", rfEncoder.getPosition());

		SmartDashboard.putNumber("lf Velocity", lfEncoder.getVelocity());
		SmartDashboard.putNumber("rf Velocity", rfEncoder.getVelocity());
	}
}
