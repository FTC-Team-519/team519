package org.ftcTeam.opmodes.registrar1;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.ftcbootstrap.ActiveOpMode;
import org.ftcbootstrap.components.TimerComponent;
import org.ftcbootstrap.components.operations.motors.GamePadTankDrive;

/**
 * Note:  It is assumed that the proper registry is used for this set of demos. To confirm please
 * search for "Enter your custom registry here"  in  {@link org.ftcTeam.FTCTeamControllerActivity}
 * <p/>
 * Summary:  Use an Operation class to perform a tank drive using the gamepad joysticks.
 * See: {@link GamePadTankDrive}
 */

@Autonomous
public class AutonomousTestPlays extends ActiveOpMode {

    TimerComponent timer;
    boolean goForward;

    private int step;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;


    /**
     * Implement this method to define the code to run when the Init button is pressed on the Driver station.
     */
    @Override
    protected void onInit() {
        getTelemetryUtil().addData("Init", getClass().getSimpleName() + " onInit.");
        getTelemetryUtil().sendTelemetry();

        frontLeft = hardwareMap.dcMotor.get("motor4");
        frontRight = hardwareMap.dcMotor.get("motor2");
        backLeft = hardwareMap.dcMotor.get("motor3");
        backRight = hardwareMap.dcMotor.get("motor1");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);


    }

    @Override
    protected void onStart() throws InterruptedException {
        super.onStart();

        step = 0;

        getTelemetryUtil().addData("Start", getClass().getSimpleName() + " onStart.");
        getTelemetryUtil().sendTelemetry();

        timer = getTimer();
    }

    /**
     * Implement this method to define the code to run when the Start button is pressed on the Driver station.
     * This method will be called on each hardware cycle just as the loop() method is called for event based Opmodes
     *
     * @throws InterruptedException
     */
    @Override
    protected void activeLoop() throws InterruptedException {
        /*
        getTelemetryUtil().addData("activeLoop", "stuff");
        //curve to first beacon
        switch(step) {
            case 0:
                moveForward(-0.3);
                curveLeft(.1);
                ++step;
                break;
            case 1:
                if(gearsTarget.isVisible())
                {

                    ++step;

                }
                break;
            case 2:
                if(atDesiredPosition()) {
                    ++step;
                }
                else {
                    keepTryingToGetAtDesiredLocation();
                }
                break;
            case 3:
                moveForward(.2);
                if(.3 seconds have passed())
        }
        moveForward(.3);
        curveLeft(.1);
        //find appropriate image
        //move in front of image
        moveForward(.1);
        //wait about .2 seconds
        moveForward(-.1);
        //wait about .3 seconds
        moveForward(0.0);
        //check if

// code goes here to test

        shoot(0.7);
        moveForward(0.2);
        //timer.setToZero; //find correct method
        //after about one to two seconds, stop; to hit the cap ball
        if(timer.targetReached(1.0)){
            moveForward(0.0);}
        }
*/
//        switch(step) {
//            case 0:
//                if (someConditionalTrue) {
//                    ++step;
//                }
//                break;
//            case 1:
//                ++step;
//                break;
//            default:
//                // should never hit here
//        }

        getTelemetryUtil().sendTelemetry();
    }
    //strafes to the right at a given rate
    private void strafeRight(double speed){
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);
    }

    //sets all of the motors to move forward at one given speed
    private void moveForward(double speed){
        /*
        frontLeft = speed;
        frontRight = speed;
        backLeft = speed;
        backRight = speed;
        */
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);
    }

    //rotates the robot to the right at a given rate
    private void rotateClockWise(double speed){
        /*
        frontLeft = speed;
        frontRight = -speed;
        backLeft = speed;
        backRight = -speed;
        */
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(speed);
        backRight.setPower(-speed);
    }

    //curves to the right
    private void curveRight(double diff){
        /*
        frontRight -= diff;
        backRight -= diff;
        */
        frontRight.setPower(frontRight.getPower() - diff);
        backRight.setPower(backRight.getPower() - diff);
    }

    private void curveLeft(double diff){
        /*
        frontLeft -= diff;
        backLeft -= diff;
        */
        frontLeft.setPower(frontLeft.getPower() - diff);
        backLeft.setPower(backLeft.getPower() - diff);
    }

    //this doesn't work at all but whatever
    //In theory, shoots a ball
    private void shoot(double pow){
        //int pow = correctPredeterminedPower;
        //sets motor to given speed
        //shootingMotor.setPower(pow);
        //feeds a particle to the shooter
        //insertMotorConveyorBeltThing.setPower(.5);

    }
}
