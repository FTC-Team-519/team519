package org.ftcTeam.training.level1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.ftcTeam.configurations.MotorAndServoRobot;
import org.ftcbootstrap.ActiveOpMode;
import org.ftcbootstrap.components.operations.motors.MotorToTime;
import org.ftcbootstrap.demos.beginner.MyFirstBot;

/**
 * Note: This Exercise assumes that you have used your Robot Controller App to "scan" your hardware and
 * saved the configuration named: "MyFirstBot" and creating a class by the same name: {@link MyFirstBot}.
 * <p/>
 * Summary
 * Demonstrates the use of a reusable "bootstrap" operation (MotorToTime) to reduce the code in the opmode.
 */

@Autonomous
public class RunMotorForTime extends ActiveOpMode {

    private MotorAndServoRobot robot;
    private MotorToTime motorToTime;

    /**
     * Implement this method to define the code to run when the Init button is pressed on the Driver station.
     */
    @Override
    protected void onInit() {

        robot = MotorAndServoRobot.newConfig(hardwareMap, getTelemetryUtil());

        motorToTime = new MotorToTime("motor with timer" , this ,robot.motor1 );

        getTelemetryUtil().addData("Init", getClass().getSimpleName() + " initialized.");
        getTelemetryUtil().sendTelemetry();

    }

    /**
     * Implement this method to define the code to run when the Start button is pressed on the Driver station.
     * This method will be called on each hardware cycle just as the loop() method is called for event based Opmodes
     * @throws InterruptedException
     */
    @Override
    protected void activeLoop() throws InterruptedException {

            //run full power (1)  for 3 seconds
            boolean hasTimeExpired = motorToTime.runToTarget(1, 3);

            if (hasTimeExpired) {
                setOperationsCompleted();
            }

        }


}
