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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.LoadShaftCommand;
import frc.robot.commands.ToggleShaftHeight;
import frc.robot.subsystems.ShaftSubsystem;

/**
 * Add your docs here.
 */


public class ShaftMechanism {
   public Joystick driver;
    
     public ShaftSubsystem shaftClimber;
    public CANSparkMax shaftMotor; 
   
    public DoubleSolenoid shaftLifter;
    
     public DigitalInput beamBreakIntake;
    

    // //buttons
    public Button buttonB;
     public Button driverLB;

    
    public ShaftMechanism( Joystick driver,  Joystick operator) {
        this.driver = driver;

        this.shaftMotor = new CANSparkMax(8, MotorType.kBrushless);


        this.buttonB = new JoystickButton(driver, 2);
       // this.driverLB = new JoystickButton(driver, 5);
       // shaftLifter = new DoubleSolenoid(ShaftConstants.fwdChannel, ShaftConstants.revChannel);
         beamBreakIntake = new DigitalInput(0);
        shaftClimber = new ShaftSubsystem(beamBreakIntake, shaftMotor, driver);
          
       // driverLB.whenPressed(new ToggleShaftHeight(shaftClimber));
      buttonB.whenPressed(new LoadShaftCommand(shaftClimber));
        
    
    
    

    }

    public ShaftSubsystem getSubsystem(){
            return shaftClimber;
    }
  
}

