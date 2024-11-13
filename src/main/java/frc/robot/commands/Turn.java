// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Turn extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_subsystem;
    public double yaw;
    public double heading;
    public final double tolerance = 1;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public Turn(DriveSubsystem subsystem, double degrees) {
        m_subsystem = subsystem;
        heading = degrees;
        m_subsystem.gyro.reset();
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        yaw = m_subsystem.gyro.getYaw();
        if (yaw < heading - tolerance || yaw > heading + tolerance) {
            // formatting diff

            double diff = (heading - yaw) / (heading * 2);
            diff = diff > 1 ? 1 : diff;
            diff = diff < -1 ? -1 : diff;
            diff = Math.abs(diff) < .3 ? Math.signum(diff) * .3 : diff;
            m_subsystem.arcade(diff, 0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !(yaw < heading - tolerance || yaw > heading + tolerance);
    }
}
