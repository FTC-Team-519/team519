package org.ftcTeam.training.level1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.ftcTeam.configurations.MotorAndServoRobot;
import org.ftcbootstrap.ActiveOpMode;
import org.ftcbootstrap.components.ServoComponent;
import org.ftcbootstrap.demos.beginner.MyFirstBot;

/**
 * Note: This Exercise assumes that you have used your Robot Controller App to "scan" your hardware and
 * saved the configuration named: "MyFirstBot" and creating a class by the same name: {@link MyFirstBot}.
 * <p/>
 * Summary
 * Demonstrates the use of a reusable "bootstrap" operation (MotorToTime) to reduce the code in the opmode.
 */

@Autonomous
public class MoveServoToPosition extends ActiveOpMode {

    private MotorAndServoRobot robot;
    private ServoComponent servoComponent;

    /**
     * Implement this method to define the code to run when the Init button is pressed on the Driver station.
     */
    @Override
    protected void onInit() {

        robot = MotorAndServoRobot.newConfig(hardwareMap, getTelemetryUtil());

        double initialPosition = 0.3;
        this.servoComponent = new ServoComponent(this,  robot.servo1,   initialPosition);


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

            double targetPosition  = 0.7;
            servoComponent.updateServoTargetPosition(targetPosition);

        }


}
