// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class DriveCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_subsystem;
    private boolean xbox = false;
    private boolean tank = false;
    private boolean init = false;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public DriveCommand(DriveSubsystem subsystem) {
        m_subsystem = subsystem;
        tank = false;
        xbox = Constants.controller.isConnected() && !(Constants.mainStick.isConnected() && Constants.secondaryStick.isConnected());
        init = false;
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        xbox = Constants.controller.isConnected() && !(Constants.mainStick.isConnected() && Constants.secondaryStick.isConnected());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        xbox = Constants.controller.isConnected() && !(Constants.mainStick.isConnected() && Constants.secondaryStick.isConnected());
        if (tank) {
            if (!xbox) {
                m_subsystem.tank(Constants.mainStick.getY(), Constants.secondaryStick.getY());
            } else {
                m_subsystem.tank(Constants.controller.getLeftY() / 2, Constants.controller.getRightY() / 2);
            }
        } else {
            if (!xbox) {
                m_subsystem.arcade(Constants.mainStick.getX(), Constants.mainStick.getY());
            } else {
                m_subsystem.arcade(Constants.controller.getRightX()/ 2, Constants.controller.getLeftY() / 2);
            }
        }
        
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
