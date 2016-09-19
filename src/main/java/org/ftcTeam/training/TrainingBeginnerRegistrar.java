package org.ftcTeam.training;

import org.ftcTeam.training.level1.*;
import org.ftcbootstrap.BootstrapRegistrar;
import org.ftcbootstrap.demos.TelemetryTest;

/**
 * Register Op Modes
 */
public class TrainingBeginnerRegistrar extends BootstrapRegistrar {


  protected Class[] getOpmodeClasses() {
    Class[] classes = {

            GamePadMotorAndServo.class,
            RunMotorForTime.class,
            MoveServoToPosition.class,
            RunMotorToEncoderTarget.class,
            TelemetryTest.class

    };

    return classes;

  }
}
