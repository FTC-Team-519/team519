package org.ftcTeam.training.level1;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.ftcTeam.configurations.MotorAndServoRobot;
import org.ftcbootstrap.ActiveOpMode;
import org.ftcbootstrap.components.operations.motors.MotorToEncoder;
import org.ftcbootstrap.components.utils.MotorDirection;
import org.ftcbootstrap.demos.beginner.MyFirstBot;

/**
 * Note: This Exercise assumes that you have used your Robot Controller App to "scan" your hardware and
 * saved the configuration named: "MyFirstBot" and creating a class by the same name: {@link MyFirstBot}.
 * <p/>
 * Summary
 * Demonstrates the use of a reusable "bootstrap" operation (MotorToTime) to reduce the code in the opmode.
 */

@Autonomous
public class RunMotorToEncoderTarget extends ActiveOpMode {

    private MotorAndServoRobot robot;
    private MotorToEncoder motorToEncoder;

    boolean targetReached;

    /**
     * Implement this method to define the code to run when the Init button is pressed on the Driver station.
     */
    @Override
    protected void onInit() {

        robot = MotorAndServoRobot.newConfig(hardwareMap, getTelemetryUtil());
        motorToEncoder = new MotorToEncoder(  this, robot.motor1);

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

            MotorDirection direction = MotorDirection.MOTOR_FORWARD;
            DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;
            //run full power (1)  move forward for and 8000 encoder count
            targetReached = motorToEncoder.runToTarget(1, 8000,direction,mode);

            if (targetReached) {
                setOperationsCompleted();
            }

        }

}
