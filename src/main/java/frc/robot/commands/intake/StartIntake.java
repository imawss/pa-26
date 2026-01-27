package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class StartIntake extends SequentialCommandGroup {
    
    public StartIntake(IntakeSubsystem intake, double waitTime, double speed) {
        addCommands(
            new ExtendIntake(intake),
            new WaitUntilCommand(intake::atTarget),
            new RunCommand(() -> intake.setRollerSpeed(speed), intake)
        );
    }
}