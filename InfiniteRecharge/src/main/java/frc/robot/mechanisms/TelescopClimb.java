/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ClimbArm1;
import frc.robot.commands.ClimbArm2;
import frc.robot.commands.ClimbWinch1;
import frc.robot.commands.ClimbWinch2;


/**
 * Add your docs here.
 */
public class TelescopClimb {
    public CANSparkMax armMotor1;
    public CANSparkMax armMotor2;
    public CANSparkMax winchMotor1;
    public CANSparkMax winchMotor2;
    public Encoder winchEncoder1;
    public Encoder winchEncoder2;
    public Joystick driver;
    public JoystickButton armBtn1;
    public JoystickButton armBtn2;
    public JoystickButton winchBtn1;
    public JoystickButton winchBtn2;

    public TelescopClimb(){
        armMotor1 = new CANSparkMax(TelescopConstants.arm1Port, MotorType.kBrushless);
        armMotor2 = new CANSparkMax(TelescopConstants.arm2Port, MotorType.kBrushless);

        winchMotor1 = new CANSparkMax(TelescopConstants.winch1Port, MotorType.kBrushless);
        winchMotor2 = new CANSparkMax(TelescopConstants.winch2Port, MotorType.kBrushless);

        winchEncoder1 = new Encoder(TelescopConstants.encoderPort1, TelescopConstants.encoderPort2);
        winchEncoder2 = new Encoder(TelescopConstants.encoderPort3, TelescopConstants.encoderPort4);

        armMotor1.setSmartCurrentLimit(20);
        armMotor2.setSmartCurrentLimit(20);
        winchMotor1.setSmartCurrentLimit(20);
        winchMotor2.setSmartCurrentLimit(20);
        
        armMotor1.setInverted(true);
        armMotor2.setInverted(true);
        
        armBtn1 = new JoystickButton(driver, 2);
        armBtn2 = new JoystickButton(driver, 3);
        winchBtn1 = new JoystickButton(driver, 4);
        winchBtn2 = new JoystickButton(driver, 5);

        armBtn1.whileHeld(new ClimbArm1());
        armBtn2.whileHeld(new ClimbArm2());
        winchBtn1.whileHeld(new ClimbWinch1());
        winchBtn2.whileHeld(new ClimbWinch2());
    }


}
