package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arm_Constants;
import frc.robot.Subsystems.Arm;


public class Arm_Commands{
     Arm arm;
        public Arm_Commands(Arm arm) {
            this.arm = arm;
        }
    public Command moveArmUp() {
            return new InstantCommand(() -> Arm.setArmUp(), arm);
    }
    public Command moveArmDown() {
        return new InstantCommand(() -> Arm.setArmDown(), Arm);
    }
    public Command l1pose(){
        return new InstantCommand(() ->arm.setGoal(Arm_Constants.goal1), arm);
    }

}
