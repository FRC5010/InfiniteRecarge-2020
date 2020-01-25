/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.dynasty;

/**
 * Add your docs here.
 */
public class ColorMap {
    private int kRedPos;
    private int kBluePos;
    private int kGreenPos;
    private int kYellowPos;
    
    public ColorMap(String color){
        //These colors are for clockwise rotations
        if(color.equalsIgnoreCase("B")){
            kRedPos = 2;
            kBluePos = 0;
            kGreenPos = 1;
            kYellowPos =3;
            
        } else if(color.equalsIgnoreCase("Y")){
            kRedPos = 3;
            kBluePos = 1;
            kGreenPos = 2;
            kYellowPos = 0;

        } else if(color.equalsIgnoreCase("R")){
            kRedPos = 0;
            kBluePos = 2;
            kGreenPos = 3;
            kYellowPos =1;

        } else if(color.equalsIgnoreCase("G")){
            kRedPos = 1;
            kBluePos = 3;
            kGreenPos = 0;
            kYellowPos = 2;

        }

    }

    public int getPos(String color){
        if(color == "Red"){
            return kRedPos;
        }else if(color == "Blue"){
            return kBluePos;
        }else if(color == "Green"){
            return kGreenPos;
        }else if(color == "Yellow"){
            return kYellowPos;
        }else{
            return 420;
            //might be uh oh
        }
    }

    // public int getRedPos(){
    //     return kRedPos;
    // }
    // public int getBluePos(){
    //     return kBluePos;
    // }
    // public int getGreenPos(){
    //     return kGreenPos;
    // }
    // public int getYellowPos(){
    //     return kYellowPos;
    // }

}
