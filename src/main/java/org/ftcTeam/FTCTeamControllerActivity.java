package org.ftcTeam;

import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;

import org.ftcbootstrap.demos.beginner.MyFirstBotRegistry;
import org.ftcbootstrap.demos.demobot.DemoBotRegistry;
import org.ftcbootstrap.demos.onemc.OneMCRegistry;
import org.ftcbootstrap.demos.pushbot.PushBotRegistry;
import org.ftcbootstrap.demos.navbot.NavBotRegistry;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

public class FTCTeamControllerActivity extends FtcRobotControllerActivity {


  @Override
  protected OpModeRegister createOpModeRegister() {

   return new FTCTeamRegistry();


  }


}
