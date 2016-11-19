package org.ftcTeam.opmodes;

import org.ftcTeam.opmodes.registrar1.AirAndScareMecanumOpMode;
import org.ftcTeam.opmodes.registrar1.AutonomousFirstTry;
import org.ftcTeam.opmodes.registrar1.AutonomousTestPlays;
import org.ftcTeam.opmodes.registrar1.AutonomousVexMotorAsServo;
import org.ftcTeam.opmodes.registrar1.AutonomousVuforia;
import org.ftcTeam.opmodes.registrar1.GamePadDriveOpMode;
import org.ftcTeam.opmodes.registrar1.GamePadMecanumOpMode;
import org.ftcTeam.opmodes.registrar1.IRFollow;
import org.ftcTeam.opmodes.registrar1.TestShooterSpeed;
import org.ftcbootstrap.ActiveOpMode;
import org.ftcbootstrap.BootstrapRegistrar;
import org.ftcbootstrap.demos.TelemetryTest;


/**
 * Register Op Modes
 */
public class Registrar1 extends BootstrapRegistrar {


  protected Class[] getOpmodeClasses() {
    Class[] classes = {

            GamePadDriveOpMode.class,
            GamePadMecanumOpMode.class,
            AirAndScareMecanumOpMode.class,
            TelemetryTest.class,
            AutonomousFirstTry.class,
            AutonomousVuforia.class,
            AutonomousVexMotorAsServo.class,
            AutonomousTestPlays.class,
            TestShooterSpeed.class,
            IRFollow.class
    };

    return classes;

  }
}