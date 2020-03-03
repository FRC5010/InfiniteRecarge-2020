/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spinner extends SubsystemBase {
  private SpeedController spinner;
  private Solenoid spinDeployment;
  private boolean isDeployed;

  ShaftSubsystem shaftSubsystem;
  /**
   * Creates a new Spinner.
   */
  public Spinner(SpeedController spinner, int fwdChannel, int revChannel, Solenoid spinDeployment, ShaftSubsystem shaftSubsystem) {
    this.spinner = spinner;
    this.spinDeployment = spinDeployment;
    this.shaftSubsystem = shaftSubsystem;
    //Set as false since spinner will not be deployed at start.
    isDeployed = false;

    //spinDeployment = new DoubleSolenoid(fwdChannel, revChannel);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spin(Double power){
    spinner.set(power);
  }

  public void stop(){
    spinner.set(0);
  }

  public void toggleSpinner(){
    if(!isDeployed){
      if(!(shaftSubsystem.isExtended())) {
        deploy();
      }
    }else{
      retract();
    }
  }

  public void deploy(){
     spinDeployment.set(true);
     isDeployed = true;
  }

  public void retract(){
    spinDeployment.set(false);
    isDeployed = false;
  }
  public boolean isDeployed(){
    return isDeployed;
  }

}
