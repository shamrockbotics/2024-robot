// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs.LEDConstants;
import frc.robot.subsystems.LEDs.LEDs;

public class LEDManager extends Command {
  /** Creates a new LEDManager. */
  int count = 0;

  double color = 1;

  private LEDs leds;
  private boolean hasNote = false;
  private boolean up = true;

  public LEDManager(LEDs leds, boolean hasNote) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hasNote = hasNote;
    this.leds = leds;
    addRequirements(leds);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // defaultColors();
    // leds.custom();
    alternateGreenGold();
  }

  public void defaultColors() {
    count++;
    // leds.setAllColor(LEDConstants.green);
    if (count > 10) {
      int index = (int) (Math.random() * (LEDConstants.colors.length - 1));
      // leds.setAllColor(LEDConstants.colors[index]);
      leds.setAllColor(new int[] {0, (int) color, 0});
      count = 0;
    }

    if (color > 20) {
      up = false;
    } else if (color < 2) {
      up = true;
    }

    if (up) {
      color += 0.1;
    } else if (!up) {
      color -= 0.1;
    }
  }

  double lastUpdate = Timer.getFPGATimestamp();
  double updateRate = 1.0;
  boolean toggle = true;

  public void alternateGreenGold() {
    int[] green = {0, 30, 0};
    int[] gold = {20, 20, 0};

    if (Timer.getFPGATimestamp() > (lastUpdate + updateRate)) {
      if (toggle) {
        leds.setPanelColor(leds.getLeftPanel(), green);
        leds.setPanelColor(leds.getRightPanel(), gold);
      } else {
        leds.setPanelColor(leds.getLeftPanel(), gold);
        leds.setPanelColor(leds.getRightPanel(), green);
      }
      lastUpdate = Timer.getFPGATimestamp();
      toggle = !toggle;
    }
    leds.update();
  }

  /*
   * public void controlLEDs(){
   * switch (getState()) {
   * case "DISABLED":
   * count++;
   * if (count > 10){
   * leds.dimRainbow();
   * count = 0;
   * }
   * break;
   * case "AUTONRED":
   * leds.setAllPanelColor(LEDConstants.red);
   * break;
   * case "AUTONBLUE":
   * leds.setAllPanelColor(LEDConstants.blue);
   * break;
   * case "TELEOP":
   * leds.togglePanel(leds.getRightPanel(), leds.getLeftPanel(),
   * LEDConstants.shamrock.get());
   * break;
   * case "SETGREEN":
   * leds.setAllColor(LEDConstants.green);
   * break;
   * case "BLINKGREEN":
   * count++;
   * if (count < 20){
   * leds.setAllColor(LEDConstants.green);
   * } else if (count >= 20){
   * leds.setAllOff();
   * }
   * if (count > 40){
   * count = 0;
   * }
   * break;
   * case "OFF":
   * leds.setAllOff();
   * break;
   * default:
   * leds.setAllOff();
   * break;
   * }
   * }
   */
  /*
  private String getState() {
    if (!DriverStation.isDSAttached()) {
      return "OFF";
    } else if (DriverStation.isDisabled()) {
      return "SETGREEN";
    } else if (hasNote) {
      return "BLINKGREEN";
    } else if (DriverStation.isAutonomous()) {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        return "AUTONBLUE";
      } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        return "AUTONRED";
      } else return "SETGREEN";
    } else {
      return "TELEOP";
    }
  }*/

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
