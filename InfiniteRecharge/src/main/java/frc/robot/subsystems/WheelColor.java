/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.dynasty.ColorMap;
import frc.robot.mechanisms.SpinConstants;

public class WheelColor extends SubsystemBase {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  // Color hashMap for position on color wheel
  private HashMap<String, ColorMap> colorMapping = new HashMap<String, ColorMap>();
  private String colorString;
  private Color currColor;

  private Color oriColor;
  private int clrCounter = 0;
  private ColorMap targetColorMap;
  private Color targetColor;

  // Used to keep track how many times each color passes.
  private int redColorCounter = 0;
  private int blueColorCounter = 0;
  private int yellowColorCounter = 0;
  private int greenColorCounter = 0;

  private Map<Color, Integer> colorCounts = new HashMap<>();
  
  /**
   * Creates a new WheelColor.
   */
  public WheelColor() {
    //Sets it to empty so it can be initialized (gets angry if not set upon initialization).
    targetColorMap = new ColorMap("");

    m_colorMatcher.addColorMatch(SpinConstants.kBlueTarget);
    m_colorMatcher.addColorMatch(SpinConstants.kGreenTarget);
    m_colorMatcher.addColorMatch(SpinConstants.kRedTarget);
    m_colorMatcher.addColorMatch(SpinConstants.kYellowTarget);

    // Initialize the color map class
    colorMapping.put("R", new ColorMap("R"));
    colorMapping.put("B", new ColorMap("B"));
    colorMapping.put("G", new ColorMap("G"));
    colorMapping.put("Y", new ColorMap("Y"));

    colorCounts.put(SpinConstants.kBlueTarget, 0);
    colorCounts.put(SpinConstants.kGreenTarget, 0);
    colorCounts.put(SpinConstants.kRedTarget, 0);
    colorCounts.put(SpinConstants.kYellowTarget, 0);    
  }

  @Override
  public void periodic() {
    final Color detectedColor = m_colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */

    // String colorString;
    final ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == SpinConstants.kBlueTarget && debounce(match.confidence)) {
      colorString = "Blue";
      currColor = match.color;
    } else if (match.color == SpinConstants.kRedTarget && debounce(match.confidence)) {
      currColor = match.color;
      colorString = "Red";
    } else if (match.color == SpinConstants.kGreenTarget && debounce(match.confidence)) {
      currColor = match.color;
      colorString = "Green";
    } else if (match.color == SpinConstants.kYellowTarget && debounce(match.confidence)) {
      colorString = "Yellow";
      currColor = match.color;
    } else {
      colorString = "Unknown";
    }

    checkColorChange(colorString, match.color);

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the sensor.
     */
    // the below code is to add the exact color onto smart dashboard
    // use the below code mostly only for testing not comp
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    // end exact code

    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);

    SmartDashboard.putNumber("Color has changed", clrCounter);

    SmartDashboard.putNumber("Red Count", redColorCounter);
    SmartDashboard.putNumber("Blue Count", blueColorCounter);
    SmartDashboard.putNumber("Green Count", greenColorCounter);
    SmartDashboard.putNumber("Yellow Count", yellowColorCounter);

    SmartDashboard.putBoolean("Find Target Color", findTargetColor());

    SmartDashboard.putNumber("distance (wheelcolor)", determineGameData());
    // SmartDashboard.putBoolean("Adjust Color", adjustToTargetColor());
    // below code is only for the exact values of the color instead of asking the
    // code to tell what the code is
    // Color detectedColor = m_colorSensor.getColor();

    // double IR = m_colorSensor.getIR();

    // SmartDashboard.putNumber("Red", detectedColor.red);
    // SmartDashboard.putNumber("Green", detectedColor.green);
    // SmartDashboard.putNumber("Blue", detectedColor.blue);
    // SmartDashboard.putNumber("IR", IR);
  }

  public int determineGameData() {
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    int distance = 0;
    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
      case 'B':
        targetColor = SpinConstants.kBlueTarget;
        // target color will be used in the functions below that will cause the wheel to
        // turn
        // this is just a way for us to store the value from the driverstation
        targetColorMap = colorMapping.get("B");
        SmartDashboard.putNumber("Target Color Distance", targetColorMap.getPos(colorString));
        break;
      case 'G':
        targetColor = SpinConstants.kGreenTarget;
        targetColorMap = colorMapping.get("G");
        SmartDashboard.putNumber("Target Color", targetColorMap.getPos(colorString));
        break;
      case 'R':
        targetColor = SpinConstants.kRedTarget;
        targetColorMap = colorMapping.get("R");
        SmartDashboard.putNumber("Target Color", targetColorMap.getPos(colorString));
        break;
      case 'Y':
        targetColor = SpinConstants.kYellowTarget;
        targetColorMap = colorMapping.get("Y");
        SmartDashboard.putNumber("Target Color", targetColorMap.getPos(colorString));
        break;
      }
    } else {
      return -1;
      // System.out.println("Wait");
    }
    distance = targetColorMap.getPos(colorString);
    return distance;
  }

  public boolean debounce(final double matchNum) {
    if (matchNum > 0.92) {
      return true;
    }
    return false;
  }

  public boolean findTargetColor() {
    if (targetColor != currColor) {
      return false;
    } else {
      return true;
    }
  }

  //Redundant for now, can be set within the SpinForNDetections command.
  // public boolean adjustToTargetColor() {
  //   if (clrCounter <= 2) { // colorMapping.get("B").getPos(colorString)){
  //     // System.out.println("gaming");
  //     clrCounter++;
  //     return false;

  //   } else {
  //     return true;
  //   }
  // }

  public int getTotalCount() {
    return clrCounter;
  }

  public int setColorCount(int num){
    clrCounter = num;
    return clrCounter;
  }
  
  //May be redundant, use getTotalCount() for now.
  // public int getColorCount(Color color) {
  //   return colorCounts.get(color);
  // }

  public boolean checkColorChange(String colorString, Color color) {
    if (currColor != oriColor && colorString != "Unknown") {
      oriColor = currColor;
      clrCounter++;

      int count = colorCounts.get(color);
      colorCounts.put(color, ++count);
      
      // Used to count how many times each color has passed.
      if (currColor == SpinConstants.kRedTarget) {
        redColorCounter++;
      } else if (currColor == SpinConstants.kBlueTarget) {
        blueColorCounter++;
      } else if (currColor == SpinConstants.kGreenTarget) {
        greenColorCounter++;
      } else if (currColor == SpinConstants.kYellowTarget) {
        yellowColorCounter++;
      }

      return true;
      // SmartDashboard.putNumber(key, value)

    }
    return false;
  }

}
