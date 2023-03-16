package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {
    // Define CAN bus ports for motor controllers
    private static final int kLeftMasterPort = 3;
    private static final int kLeftFollowerPort = 1;
    private static final int kRightMasterPort = 12;
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

    //Initialize the turning 180 degrees time duration
    private static final double turnDuration = 2.0;

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
        m_leftFollower.follow(m_leftMaster);
        m_rightFollower.follow(m_rightMaster);

        // Create motor controller groups for the left and right side of the robot
        m_right = new MotorControllerGroup(m_leftMaster, m_leftFollower);
        m_left = new MotorControllerGroup(m_rightMaster, m_rightFollower);

        // Create differential drive object with the left and right motor controller
        // groups
        m_drive = new DifferentialDrive(m_right, m_left);

        // Set the idle mode for all motor controllers to brake
        m_leftMaster.setIdleMode(IdleMode.kBrake);
        m_leftFollower.setIdleMode(IdleMode.kBrake);
        m_rightMaster.setIdleMode(IdleMode.kBrake);
        m_rightFollower.setIdleMode(IdleMode.kBrake);

        // Set the left motor controller to reverse direction
        m_leftMaster.setInverted(true);
    }

    // Runs periodically during the teleoperated (driver-controlled) period
    @Override
    public void teleopPeriodic() {
        // Get joystick input for forward/backward movement and turning
        double move = -m_driverController.getRawAxis(1);
        double turn = m_driverController.getRawAxis(4);

        // Apply deadband to joystick input
        move = Math.abs(move) > kDeadband ? move : 0;
        turn = Math.abs(turn) > kDeadband ? turn : 0;

        // Apply turbo
        if (m_driverController.getRawButtonPressed(2)) {
            move *= 2;
        }
        
        // Turn the robot 180 degrees
        if (m_driverController.getRawButtonPressed(3)) {
            m_leftMaster.set(0.5);
            m_rightMaster.set(0.5);
            Timer.delay(turnDuration);
            m_leftMaster.stopMotor();
            m_rightMaster.stopMotor();
        }

        // Drive the robot with the joystick inputs
        m_drive.arcadeDrive(move, turn);
        m_drive.arcadeDrive(move, turn);
        // Send some telemetry to the dashboard
        SmartDashboard.putNumber("Move", move);
        SmartDashboard.putNumber("Turn", turn);
    }
}
