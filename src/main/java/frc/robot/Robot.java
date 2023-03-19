package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;

public class Robot extends TimedRobot {
    // Define CAN bus ports for motor controllers
    private static final int kLeftMasterPort = 3;
    private static final int kLeftFollowerPort = 12;
    private static final int kRightMasterPort = 1;
    private static final int kRightFollowerPort = 2;

    // Define deadband for joystick input
    private static final double kDeadband = 0.1;

    // Initialize joystick object
    private Joystick m_driverController;

    // Initialize motor controller objects
    private CANSparkMax m_leftMaster;
    private CANSparkMax m_leftFollower;
    private CANSparkMax m_rightMaster;
    private CANSparkMax m_rightFollower;

    // Initialize differential drive object
    private DifferentialDrive m_drive;

    // Initialize motor controller groups for left and right side of robot
    MotorControllerGroup m_right;
    MotorControllerGroup m_left;

    // Initialize the turning 180 degrees time duration
    private static final double turnDuration = 6.0;

    // Initializing SlewRateLimiters for acceleration
    private final SlewRateLimiter moveLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter turnLimiter = new SlewRateLimiter(3);

    // Initializing a PID Controller
    private PIDController pidController;

    // Runs once when the robot is turned on
    @Override
    public void robotInit() {
        // Create joystick object for driver controller
        m_driverController = new Joystick(0);

        // Initialize motor controllers with their respective CAN bus ports
        m_leftMaster = new CANSparkMax(kLeftMasterPort, MotorType.kBrushed);
        m_leftFollower = new CANSparkMax(kLeftFollowerPort, MotorType.kBrushed);
        m_rightMaster = new CANSparkMax(kRightMasterPort, MotorType.kBrushed);
        m_rightFollower = new CANSparkMax(kRightFollowerPort, MotorType.kBrushed);

        // Set the follower motors to follow their respective master motor

        // Create motor controller groups for the left and right side of the robot
        m_right = new MotorControllerGroup(m_leftMaster, m_leftFollower);
        m_left = new MotorControllerGroup(m_rightMaster, m_rightFollower);

        // Create differential drive object with the left and right motor controller
        // groups

        m_leftFollower.follow(m_leftMaster);
        m_rightFollower.follow(m_rightMaster);

        // Set the idle mode for all motor controllers to brake
        m_leftMaster.setIdleMode(IdleMode.kCoast);

        m_rightMaster.setIdleMode(IdleMode.kCoast);

        // Set the left motor controller to reverse direction
        m_leftMaster.setInverted(true);
        m_leftFollower.setInverted(true);

        m_rightMaster.setInverted(false);
        m_rightFollower.setInverted(false);

        m_drive = new DifferentialDrive(m_left, m_right);

    }

    // Runs periodically during the teleoperated (driver-controlled) period
    @Override
    public void teleopPeriodic() {
        // Get joystick input for forward/backward movement and turning
        double move = 0;
        double turn = 0;
        // Main driving for drifting
        if (Math.abs(m_driverController.getRawAxis(2)) > 0.75) {
            move = -m_driverController.getRawAxis(1);
            turn = m_driverController.getRawAxis(4) * 0.7;
        } 
        // Regular Speed
        else {
            move = -m_driverController.getRawAxis(1) * 0.5;
            turn = m_driverController.getRawAxis(4) * 0.65;
        }

        // Apply deadband to joystick input
        move = Math.abs(move) > kDeadband ? move : 0;
        turn = Math.abs(turn) > kDeadband ? turn : 0;

        // Makes the driving smoother using SlewRateLimiters
        move = moveLimiter.calculate(move);
        turn = turnLimiter.calculate(turn);
        /*
         * You can times this by DriveConstants.MaxSpeedMetersPerSecond
         * or DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond
        */
        // Drive the robot with the joystick inputs
        if (m_driverController.getRawButtonPressed(3)) {
            m_drive.arcadeDrive(0, 0.5);
            Timer.delay(2.0);
            m_drive.stopMotor();
        } else {
            m_drive.arcadeDrive(move, turn);
        }

        // Send some telemetry to the dashboard
        SmartDashboard.putNumber("Move", move);
        SmartDashboard.putNumber("Turn", turn);
    }

    @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}
}
