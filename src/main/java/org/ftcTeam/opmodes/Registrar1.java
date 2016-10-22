package org.ftcTeam.opmodes;

import org.ftcTeam.opmodes.registrar1.AirAndScareMecanumOpMode;
import org.ftcTeam.opmodes.registrar1.GamePadDriveOpMode;
import org.ftcTeam.opmodes.registrar1.GamePadMecanumOpMode;
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
            TelemetryTest.class

    };

    return classes;

  }
}