/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.subsystems.Climber;
/**
 * Add your docs here.
 */
public class ClimberConstants {
    public DoubleSolenoid topSolenoid;
    public DoubleSolenoid bottomSolenoid;
    public CANSparkMax winchMotor;
    public Climber climberMain;

    ClimberConstants(){
        topSolenoid = new DoubleSolenoid(4, 5);
        bottomSolenoid = new DoubleSolenoid(6, 7);
        winchMotor = new CANSparkMax(6, MotorType.kBrushless);

        climberMain = new Climber(topSolenoid, bottomSolenoid, winchMotor);
    }
    
}
