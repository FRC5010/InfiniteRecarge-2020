/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.mechanisms;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.ControlConstants;
import frc.robot.commands.ClimbArm;
import frc.robot.commands.ClimbWinch;
import frc.robot.subsystems.TelescopSubsystem;


/**
 * Add your docs here.
 */
public class TelescopClimb {
    public CANSparkMax armMotor1;
    public CANSparkMax armMotor2;
    public CANSparkMax winchMotor1;
    public CANSparkMax winchMotor2;
    public Joystick driver;
    public Joystick operator;
    public JoystickButton armBtn;
   
    public JoystickButton winchBtn;
    public JoystickButton armOvrd;
    
    private TelescopSubsystem subsystem;

    public TelescopClimb(Joystick driver, Joystick operator){
        armMotor1 = new CANSparkMax(TelescopConstants.arm1Port, MotorType.kBrushless);
        armMotor2 = new CANSparkMax(TelescopConstants.arm2Port, MotorType.kBrushless);

        winchMotor1 = new CANSparkMax(TelescopConstants.winch1Port, MotorType.kBrushless);
        winchMotor2 = new CANSparkMax(TelescopConstants.winch2Port, MotorType.kBrushless);
        this.driver = driver;
        this.operator = operator;

        subsystem = new TelescopSubsystem(winchMotor1, winchMotor2, armMotor1, armMotor2, driver, operator);
        armMotor1.setSmartCurrentLimit(TelescopConstants.armCurrentLimit);
        armMotor2.setSmartCurrentLimit(TelescopConstants.armCurrentLimit);
        winchMotor1.setSmartCurrentLimit(TelescopConstants.winchCurrentLimit);
        winchMotor2.setSmartCurrentLimit(TelescopConstants.winchCurrentLimit);
        
        armMotor1.setInverted(false);
        armMotor2.setInverted(true);

        winchMotor2.setInverted(true);
        
        armBtn = new JoystickButton(operator, ControlConstants.startClimb );
        armOvrd = new JoystickButton(operator, ControlConstants.toggleBarrelHeight);
        winchBtn = new JoystickButton(operator, ControlConstants.retractClimb);
        

        armBtn.whileHeld(new ClimbArm(subsystem));
       armOvrd.whenPressed(new InstantCommand(()->subsystem.overrideArms = true));
        winchBtn.whileHeld(new ClimbWinch(subsystem));
        
    }


}
