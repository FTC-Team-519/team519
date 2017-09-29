package org.ftcTeam.opmodes.registrar1;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
        getTelemetryUtil().addData("activeLoop", "stuff");
        /** Comment out so that whole project builds

        //curve to first beacon
        switch(step) {
            case 0:
                curveLeft(-0.3, 0.1); //curves the robot driving backwards toward the first beacon
                ++step;
                break;
            case 1:
                if(gearsTarget.isVisible())//checks if it can see the first target/beacon
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
                if(.3secondsHavePassed()) {
                moveForward(-0.2);
                if(.3secondsHavePassed()) {
                    moveForward(0.0);
                    if(colorOfBeacon.isDesiredColor()) {
                        step++;
                        break;
                        else {
                            if(5.0secondsHavePassed()){
                                moveForward(.2);
                                if(.3secondsHavePassed()) {
                                    moveForward(-0.2);
                                    if(.3secondsHavePassed()) {
                                        moveForward(0.0);
                                        step++;
                                        break;
                                    }
                                }
                            }
                        }
                    }   }   }
            case 4:
                strafeRight(.8);//drives toward second beacon
                break;
            case 5:
                if(legosTarget.isVisible())//checks if it can see the second target/beacon
                {

                    ++step;

                }
                break;
            case 6:
                if(atDesiredPosition()) {
                    ++step;
                }
                else {
                    keepTryingToGetAtDesiredLocation();
                }
                break;

            case 7:
                moveForward(.2);
                if(.3secondsHavePassed()) {
                    moveForward(-0.2);
                    if(.3secondsHavePassed()) {
                        moveForward(0.0);
                        if(colorOfBeacon.isDesiredColor()) {
                            step++;
                            break;
                            else {
                                if(5.0secondsHavePassed()){
                                    moveForward(.2);
                                    if(.3secondsHavePassed()) {
                                        moveForward(-0.2);
                                        if(.3secondsHavePassed()) {
                                            moveForward(0.0);
                                            step++;
                                            break;
                                        }
                                    }
                                }
                            }
                }   }   }
            case 8: {
                rotateClockWise(0.6);//rotates until pointing at center vortex
                if(.4secondsHavePassed())
                {
                    moveForward(0.4);//moves toward center vortex
                    if(1.0secondsHavePassed())
                    {
                        moveForward(0.0); //stops, hopefully on center vortex
                    }
                }

            }
            }
         **/

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
    private void curveRight(double pow, double diff){
        /*
        frontRight -= diff;
        backRight -= diff;
        */
        frontLeft.setPower(pow);
        backLeft.setPower(pow);
        frontRight.setPower(frontRight.getPower() - diff);
        backRight.setPower(backRight.getPower() - diff);
    }

    private void curveLeft(double pow, double diff){
        /*
        frontLeft -= diff;
        backLeft -= diff;
        */
        frontRight.setPower(pow);
        backRight.setPower(pow);
        frontLeft.setPower(pow - diff);
        backLeft.setPower(pow - diff);
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
