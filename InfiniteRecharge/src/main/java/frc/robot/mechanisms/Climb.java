/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.mechanisms;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.mechanisms.ClimberConstants;
import frc.robot.subsystems.Climber;

/**
 * Add your docs here.
 */
public class Climb {
    public DoubleSolenoid topSolenoid;
    public DoubleSolenoid bottomSolenoid;
    public CANSparkMax winchMotor1;
    public CANSparkMax winchMotor2;
    public Climber climberMain;
    public CANPIDController climbPidController;

    public Climb(){
        topSolenoid = new DoubleSolenoid(4, 5);
        bottomSolenoid = new DoubleSolenoid(6, 7);
        winchMotor1 = new CANSparkMax(6, MotorType.kBrushless);
        winchMotor2 = new CANSparkMax(7, MotorType.kBrushless);

        //Seperate motors will be used for now.
        //winchMotor2.follow(winchMotor1);
        
        // climbPidController = winchMotor.getPIDController();
        // climbPidController.setP(ClimberConstants.kP);
        // climbPidController.setI(ClimberConstants.kI);
        // climbPidController.setD(ClimberConstants.kD);
        // climbPidController.setOutputRange(ClimberConstants.kMinOutput, ClimberConstants.kMaxOutput);

        climberMain = new Climber(topSolenoid, bottomSolenoid, winchMotor1, climbPidController);
}
}
