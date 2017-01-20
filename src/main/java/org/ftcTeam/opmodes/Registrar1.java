package org.ftcTeam.opmodes;

import org.ftcTeam.opmodes.registrar1.AirAndScareMecanumOpMode;
import org.ftcTeam.opmodes.registrar1.AutonomousCopy;
import org.ftcTeam.opmodes.registrar1.AutonomousFirstTry;
import org.ftcTeam.opmodes.registrar1.AutonomousRedDistanceSensor;
import org.ftcTeam.opmodes.registrar1.AutonomousTestPlays;
import org.ftcTeam.opmodes.registrar1.AutonomousVexMotorAsServo;
import org.ftcTeam.opmodes.registrar1.AutonomousVuforia;
import org.ftcTeam.opmodes.registrar1.AutonomousVuforiaBlue;
import org.ftcTeam.opmodes.registrar1.AutonomousWithoutVuforia;
import org.ftcTeam.opmodes.registrar1.GamePadDriveOpMode;
import org.ftcTeam.opmodes.registrar1.GamePadMecanumOpMode;
import org.ftcTeam.opmodes.registrar1.IRFollow;
import org.ftcTeam.opmodes.registrar1.NavxTest;
import org.ftcTeam.opmodes.registrar1.ServoTest;
import org.ftcTeam.opmodes.registrar1.SimpleAutonomous;
import org.ftcTeam.opmodes.registrar1.StrafeTest;
import org.ftcTeam.opmodes.registrar1.Teleop;
import org.ftcTeam.opmodes.registrar1.TeleopUltrasonicTest;
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

            //GamePadDriveOpMode.class,
            GamePadMecanumOpMode.class,
            //AirAndScareMecanumOpMode.class,
            TelemetryTest.class,
            //AutonomousFirstTry.class,
            AutonomousVuforia.class,
            AutonomousVuforiaBlue.class,
            AutonomousCopy.class,
            SimpleAutonomous.class,
            AutonomousRedDistanceSensor.class,
            StrafeTest.class,
            TeleopUltrasonicTest.class,
            //AutonomousVexMotorAsServo.class,
            //AutonomousTestPlays.class,
            Teleop.class,
            //AutonomousWithoutVuforia.class,
            NavxTest.class,
            ServoTest.class
            //TestShooterSpeed.class,
            //IRFollow.class
    };

    return classes;

  }
}