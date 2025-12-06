package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


import com.pathplanner.lib.commands.PathPlannerAuto;


public class RobotContainer {
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

    public RobotContainer() {
        configureButtonBindings();

        m_robotDrive.setDefaultCommand(
            new RunCommand(
                () -> m_robotDrive.drive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true
                ),
                m_robotDrive
            )
        );
    }

    private void configureButtonBindings() {
        new JoystickButton(m_driverController, Button.kR1.value)
            .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Start To Reef");
    }        // Assuming this is a method in your drive subsystem
}
