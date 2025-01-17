package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Arm_Constants;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Arm extends SubsystemBase{
    
    private TalonFX armMotor;
    private MotorOutputConfigs motorOutput;
    private PIDController armPID;
    private double goal = 0;
    private final DutyCycleOut request = new DutyCycleOut(0.0);
    private ArmFeedforward feedFor;
    
    private SingleJointedArmSim arm = new SingleJointedArmSim(DCMotor.getKrakenX60(1), Arm_Constants.gearRatio, Arm_Constants.jkg, Arm_Constants.armLength, Arm_Constants.min, Arm_Constants.max, true, Arm_Constants.min);

    public final SysIdRoutine sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                this::setVoltage,
                log -> {
                    log.motor("arm") 
                        .voltage(Volts.of(armMotor.get() * RobotController.getBatteryVoltage()))
                        .angularPosition(Rotations.of(getPose()))
                        .angularVelocity(RotationsPerSecond.of(getVelocity()));
                },
                this));

    private final Mechanism2d mech2d = new Mechanism2d(20, 50);
    private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Arm Root", 10, 0);
    private final MechanismLigament2d armMech2d = mech2dRoot.append(new MechanismLigament2d("Arm", 20, 90));

        
    public Arm() {
        armMotor = new TalonFX(Arm_Constants.motorId, "rhino");
        motorOutput = new MotorOutputConfigs();
        armPID = new PIDController(Arm_Constants.kP, Arm_Constants.kI, Arm_Constants.kP);
        feedFor = new ArmFeedforward(Arm_Constants.ks, Arm_Constants.kg, Arm_Constants.kv);

   
        armPID.setTolerance(Arm_Constants.tolerance);
        Shuffleboard.getTab("Arm Controller").add(armPID);
    }

    public void setArmUp() {
        motorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        armMotor.getConfigurator().apply(motorOutput);
    }
    public void setArmDown() {
        motorOutput.Inverted = InvertedValue.Clockwise_Positive;
        armMotor.getConfigurator().apply(motorOutput);
    }

    public void setSpeed(double speed) {
        armMotor.setControl(request.withOutput(speed));
    }
    
    public double getPose() {
        return ((armMotor.getPosition().getValueAsDouble()+Arm_Constants.armOffset)/Arm_Constants.gearRatio);
    }
    
    public void setGoal(double newGoal){
        goal = newGoal;
    }
    public double getGoal(){
        return goal;
    }
    public void setVoltage(Voltage v) {
        armMotor.setVoltage(v.magnitude());
    }
    public double getVelocity() {
        return armMotor.getVelocity().getValueAsDouble()/Arm_Constants.gearRatio;
    }
    public void moveToLevel(double goal) {
        armPID.calculate(goal);
    }

    @Override
    public void periodic(){
        double extra = feedFor.calculate(goal, 0);
        double voltage = armPID.calculate(getPose(), goal)+extra;
        setVoltage(Volts.of(voltage));
    }
}
