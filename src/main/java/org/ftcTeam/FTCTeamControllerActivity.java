package org.ftcTeam;

import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;

import org.ftcTeam.opmodes.*;
import org.ftcTeam.training.*;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

public class FTCTeamControllerActivity extends FtcRobotControllerActivity {


  @Override
  protected OpModeRegister createOpModeRegister() {

      //return new Registrar1();
      //return new Registrar2();
      return new TrainingBeginnerRegistrar();


  }


}
