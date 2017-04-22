package org.ftcTeam.opmodes.registrar1;

//RED
import android.util.Base64;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.ftcbootstrap.ActiveOpMode;
import org.ftcbootstrap.components.TimerComponent;

import com.qualcomm.ftcrobotcontroller.R;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import java.io.UnsupportedEncodingException;
import java.text.DecimalFormat;

/**
 * Autonomous Vuforia reference class that doesn't depend upon any motors, just the camera.
 */

@Autonomous
public class AutonomousCopy extends ActiveOpMode {
    private static final String A = "QWJaajY5di8vLy8vQUFBQUdTWHZyMEc2TDBrTXJ3TUQwT";
    private static final String B = "zdZRWdNc0lmOEI2WFZ1eEV1UTNCci8wV1d0Tk13dHBSMm";
    private static final String C = "5sZHA5cmsxTk1PbHhWc0VBcXIwbmFac1o0dmlvTlZ0R0Q";
    private static final String D = "2K2tMUXVZd0lrSWlKMXlsZGl5Wjd4U055Qkt0Yk0zeVoy";
    private static final String E = "dFBYMFRFZnYrY1o0L3d5TmhXRVoxMU0wdk9GenVsSUFlc";
    private static final String F = "mRwUTBhNzRzeTZHWWc5UzFtSzRGUDJnbFh0b2tuVy94Um";
    private static final String G = "FBaG1zWnkzaXRjZWhYZCtLYVNRS2JBNzZMOGdGcytFY2x";
    private static final String H = "RQW1XQlFwdVRIRHlLcmVMbzEvaHlxUjRzb1BaUXIxemgy";
    private static final String I = "RXg1RWZ0K01TdzBmaUh3NjhtWG1lWDh1L1ZrMk00QWVsR";
    private static final String J = "ERlSE5UdndoMHJ1dnVYczdGVmRaZDBMWm4yMnR4RStWbm";
    private static final String K = "h6cGt5OUFFbEI5eHZVYkFoZTIwN2lWSU5wSndpZDR3d0l";
    private static final String L = "LZ3VFTEdRMTQK";

    private static final String VALUE = A + B + C + D + E + F + G + H + I + J + K + L;

    private static final byte[] MY_VALUE = Base64.decode(VALUE, Base64.NO_WRAP);
    private static final String KEY = convert(MY_VALUE);

    private DecimalFormat df = new DecimalFormat("#.##");
    private boolean calibration_complete = false;
    private AHRS navx_device;

    // FIXME: This should be step 0 unless you are testing
    int step = 0;
    private boolean missedBeacon = false;

    private static String convert(byte[] thing) {
        try {
            return new String(thing, "US-ASCII");
        }
        catch (UnsupportedEncodingException uee) {
            return "Could not convert!!!";
        }
    }

    private static final double PI = 3.14159d;

    /**
     * Degrees to radians is pi times the angle in degrees divided by 180 degrees.
     * @param degrees
     * @return
     */
    public static double degreesToRadians(double degrees) {
        return PI * degrees / 180.0d;
    }

    /**
     * Returns the transformed x and y vector values to account for z rotation.
     *
     * @param x the x coordinate vector value
     * @param y the y coordinate vector value
     * @param degrees the current rotation around the z axis
     * @returntransformed x and y vector values to account for z rotation.
     */
    public static double[] rotateVector(double x, double y, double degrees) {
        double[] retValue = new double[2];

        double radians = degreesToRadians(degrees);
        double cosineA = Math.cos(radians);
        double sineA   = Math.sin(radians);

        // RotationMatrix = +cos?   -sin?
        //                  +sin?   +cos?
        //
        retValue[0] = (x * cosineA) - (y * sineA);
        retValue[1] = (x * sineA) + (y * cosineA);
        return retValue;
    }

    /**
     * Returns power values normalized across motors so that they are all within the
     * expected [-1.0 .. 1.0] range, which is often not true after combining values for mecanum.
     *
     * @param motorPowers the motor values to normalize to a max point.
     */
    private static void normalizeCombinedPowers(double[] motorPowers) {
        double maxAbsPower = 0.0d;

        for (double motorPower : motorPowers) {
            double tmpAbsPower = Math.abs(motorPower);
            if (tmpAbsPower > maxAbsPower) {
                maxAbsPower = tmpAbsPower;
            }
        }

        if (maxAbsPower > 1.0d) {
            for (int i = 0; i < motorPowers.length; ++i) {
                motorPowers[i] = motorPowers[i] / maxAbsPower;
            }
        }
    }

    /**
     * Returns a power that is reduced proportionally to the maximum speed, which would be
     * no reduction if max speed is 1.0 (or basically 100%).
     *
     * @param input power to potentially be reduced.
     * @return a power that is reduced proportionally to the maximum speed, which would be
     * no reduction if max speed is 1.0 (or basically 100%).
     */
    private static double reducePower(double input) {
        // return input;   // If no reduction desired
        return input * MAX_SPEED;
    }

    public static final float MM_PER_INCH = 25.4f;
    public static final float MM_BOT_WIDTH = 18 * MM_PER_INCH;            // ... or whatever is right for your robot
    public static final float MM_FTC_FIELD_WIDTH = (12*12 - 2) * MM_PER_INCH;   // the FTC field is ~11'10" center-to-center of the glass panels
    public static final float MM_TARGET_CENTER_HEIGHT_Z = (1.5f + (8.5f / 2.0f)) * MM_PER_INCH; // half height of 8.5x11 sheet plus 1.5" above floor
    public static final float MM_NEAR_OFFSET = (1 * 12) * MM_PER_INCH; // one foot away from center
    public static final float MM_FAR_OFFSET = (3 * 12) * MM_PER_INCH;  // three feet away from center

    public static final float MM_RED_BEACON_WALL_X = -(MM_FTC_FIELD_WIDTH/2);
    public static final float MM_BLUE_BEACON_WALL_Y = (MM_FTC_FIELD_WIDTH/2);

    public static final float MM_RED_NEAR_TARGET_Y  = -MM_NEAR_OFFSET;
    public static final float MM_RED_FAR_TARGET_Y   = MM_FAR_OFFSET;
    public static final float MM_BLUE_NEAR_TARGET_X = MM_NEAR_OFFSET;
    public static final float MM_BLUE_FAR_TARGET_X  = -MM_NEAR_OFFSET;

    // FIXME: This should be set to whatever distance is desired back from wall to look at
    //        colors/maybe shoot/and start going straight into buttons once lined up with target
    public static final float DESIRED_MM_DISTANCE_FROM_WALL = 16 * MM_PER_INCH;

    public static final float DESIRED_MM_RED_X      = MM_RED_BEACON_WALL_X + DESIRED_MM_DISTANCE_FROM_WALL;
    public static final float DESIRED_MM_RED_NEAR_Y = MM_RED_NEAR_TARGET_Y;
    public static final float DESIRED_MM_RED_FAR_Y  = MM_RED_FAR_TARGET_Y;
    public static final float DESIRED_DEGREES_RED_Z = 90f;  // compare against orientation

    public static final float DESIRED_MM_BLUE_Y      = MM_BLUE_BEACON_WALL_Y - DESIRED_MM_DISTANCE_FROM_WALL;
    public static final float DESIRED_MM_BLUE_NEAR_X = MM_BLUE_NEAR_TARGET_X;
    public static final float DESIRED_MM_BLUE_FAR_X  = MM_BLUE_FAR_TARGET_X;
    public static final float DESIRED_DEGREES_BLUE_Z = -90f; // compare against orientation

    private VuforiaLocalizer.Parameters parameters;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables ftc2016Trackables;

    private VuforiaTrackable wheels;
    private VuforiaTrackable tools;
    private VuforiaTrackable legos;
    private VuforiaTrackable gears;

    private OpenGLMatrix lastKnownLocation = null;

    private DcMotor shooter;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor topCollector;
    private DcMotor midCollector;
    private DcMotor frontCollector;
    private ColorSensor colorSensor;
    private I2cDevice ultrasonicSensor;
    public I2cDeviceSynch ultrasonicReader;

    private double[] motorPowers = new double[4];
    private static final int FRONT_LEFT  = 0;
    private static final int FRONT_RIGHT = 1;
    private static final int BACK_LEFT   = 2;
    private static final int BACK_RIGHT  = 3;

    boolean beaconIsRed = false;

    private static final double MAX_SPEED = 0.3d;

    byte[] ultrasonicCache;
    public static final int ULTRASONIC_REG_START = 0x04; //Register to start reading
    public static final int ULTRASONIC_READ_LENGTH = 2; //Number of byte to read

    // FIXME: This should be false, but change to true to not shoot during testing
    boolean firstShotComplete = false;


    /**
     * Implement this method to define the code to run when the Init button is pressed on the Driver station.
     */
    @Override
    protected void onInit() {
        // FIXME: This is where we need to get the mapping correct!!!!
        // NOTE: For "forward bumper":  [FL: motor4, FR: motor2, BL: motor3, BR: motor1]
        // NOTE: For "backward bumper": [FL: motor1, FR: motor3, BL: motor2, BR: motor4]
        frontLeft = hardwareMap.dcMotor.get("motor4");
        frontRight = hardwareMap.dcMotor.get("motor2");
        backLeft = hardwareMap.dcMotor.get("motor3");
        backRight = hardwareMap.dcMotor.get("motor1");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        topCollector = hardwareMap.dcMotor.get("topCollector");
        midCollector = hardwareMap.dcMotor.get("feeder");
        shooter = hardwareMap.dcMotor.get("shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.getController().setMotorZeroPowerBehavior(1, DcMotor.ZeroPowerBehavior.FLOAT);

        I2cAddr ultrasonicAddress = new I2cAddr(0x14);
        ultrasonicSensor = hardwareMap.i2cDevice.get("range");
        ultrasonicReader = new I2cDeviceSynchImpl(ultrasonicSensor, ultrasonicAddress, false);
        ultrasonicReader.engage();

        colorSensor = hardwareMap.colorSensor.get("colorSensor1");
        colorSensor.enableLed(false);

        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = KEY;
        //parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        ftc2016Trackables = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");

        wheels = ftc2016Trackables.get(0);
        tools  = ftc2016Trackables.get(1);
        legos  = ftc2016Trackables.get(2);
        gears  = ftc2016Trackables.get(3);

        wheels.setName("wheels"); // blue-near
        tools.setName("tools"); // red-far
        legos.setName("legos"); // blue-far
        gears.setName("gears"); // red-near

        OpenGLMatrix redNearTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                // This also translates +z to be correct height above floor
                // This translates -y to move along wall toward Red team
                .translation(MM_RED_BEACON_WALL_X, MM_RED_NEAR_TARGET_Y, MM_TARGET_CENTER_HEIGHT_Z)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        gears.setLocation(redNearTargetLocationOnField);

        OpenGLMatrix redFarTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                // This also translates +z to be correct height above floor
                // This translates +y to move along wall away from Red team
                .translation(MM_RED_BEACON_WALL_X, MM_RED_FAR_TARGET_Y, MM_TARGET_CENTER_HEIGHT_Z)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        tools.setLocation(redFarTargetLocationOnField);

        OpenGLMatrix blueNearTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                // This also translates +z to be correct height above floor
                // This translates +x to move along wall toward Blue team
                .translation(MM_BLUE_NEAR_TARGET_X, MM_BLUE_BEACON_WALL_Y, MM_TARGET_CENTER_HEIGHT_Z)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        wheels.setLocation(blueNearTargetLocationOnField);

        OpenGLMatrix blueFarTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                // This also translates +z to be correct height above floor
                // This translates -x to move along wall away from Blue team
                .translation(MM_BLUE_FAR_TARGET_X, MM_BLUE_BEACON_WALL_Y, MM_TARGET_CENTER_HEIGHT_Z)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        legos.setLocation(blueFarTargetLocationOnField);

        // NOTE: +Y translation puts it on front of robot, and -Y to put on back
        // FIXME: Still need +z translation to put at proper height (probably not centered)
        OpenGLMatrix frontCenteredLocationOnRobot = OpenGLMatrix
                .translation(0, MM_BOT_WIDTH/2, MM_BOT_WIDTH/2)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, -90, 0));

        OpenGLMatrix backCenteredLocationOnRobot = OpenGLMatrix
                .translation(0, -MM_BOT_WIDTH/2, MM_BOT_WIDTH/2)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, -90, 0));

        OpenGLMatrix phoneLocationOnRobot = frontCenteredLocationOnRobot;

        ((VuforiaTrackableDefaultListener)wheels.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)tools.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)legos.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)gears.getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        //Note The Telemetry Utility is designed to let you organize all telemetry data before sending it to
        //the Driver station via the sendTelemetry command
        getTelemetryUtil().addData("Init", getClass().getSimpleName() + " onInit.");
        getTelemetryUtil().sendTelemetry();
    }

    @Override
    protected void onStart() throws InterruptedException {
        super.onStart();

        // NOTE: Ensure motors not going anywhere at start
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);

        // NOTE: Start monitoring for images on the field
        ftc2016Trackables.activate();

        getTelemetryUtil().addData("Start", getClass().getSimpleName() + " onStart.");
        getTelemetryUtil().sendTelemetry();
    }

    /**
     * Implement this method to define the code to run when the Start button is pressed on the Driver station.
     * This method will be called on each hardware cycle just as the loop() method is called for event based Opmodes
     *
     * @throws InterruptedException
     */
    @Override
    protected void activeLoop() throws InterruptedException {
        boolean isVisible = false;
        getTelemetryUtil().addData("Step ", "" + step);
        ultrasonicCache = ultrasonicReader.read(ULTRASONIC_REG_START, ULTRASONIC_READ_LENGTH);

        if (navx_device != null) {
            if (calibration_complete) {
                getTelemetryUtil().addData("NavX", "Yaw: " + df.format(navx_device.getYaw()));
            } else {
                getTelemetryUtil().addData("NavX", "Calibrating :-(");
                calibration_complete = !navx_device.isCalibrating();
            }
        }
        getTelemetryUtil().addData("distance", ultrasonicCache[0]);
        switch(step) {
            case 0:
                forward(-0.25d);
                if (getTimer().targetReached(1.55d)) {
                //if (getTimer().targetReached(0.75d)) {

                //if (ultrasonicCache[0] < 65 && ultrasonicCache[0] > 0) {//back up until it is ten centimeters away from wall
                    stopMoving();
                    ++step;
                    //step = 999999;
                }
                break;
            case 1:
                isVisible = ((VuforiaTrackableDefaultListener)gears.getListener()).isVisible();
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)gears.getListener()).getUpdatedRobotLocation();
                getTelemetryUtil().addData("Target", "Looking for target.");
                if (robotLocationTransform != null) {
                    lastKnownLocation = robotLocationTransform;
                }

                if (lastKnownLocation != null && isVisible) {
                    Orientation orientation = Orientation.getOrientation(lastKnownLocation,
                            AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    float pErrorDegZ = DESIRED_DEGREES_RED_Z - orientation.thirdAngle;

                    double zVector = 0.0f;
                    if (pErrorDegZ < -3f) {
                        //zVector = -.16f;
                        zVector = -0.19f;
                    } else if (pErrorDegZ > -3f) {
                        //zVector = 0.16f;
                        zVector = 0.19f;
                    }

                    turnLeft(zVector, true);

                    if (Math.abs(pErrorDegZ) <= 4f) {
                    //if (pErrorDegZ > -14 && pErrorDegZ < -8){
                        stopMoving();

                        ++step;
                    }
                }
                else {
                    //turnLeft(0.12f, true);
                    turnLeft(0.13f, true);
                }

                break;
            case 2:
                isVisible = ((VuforiaTrackableDefaultListener)gears.getListener()).isVisible();
                robotLocationTransform = ((VuforiaTrackableDefaultListener)gears.getListener()).getUpdatedRobotLocation();
                getTelemetryUtil().addData("Target", "Looking for target.");
                if (robotLocationTransform != null) {
                    lastKnownLocation = robotLocationTransform;
                }

                if (lastKnownLocation != null && isVisible) {
                    getTelemetryUtil().addData("Location:", lastKnownLocation.formatAsTransform());
                    float[] xyzTranslation = lastKnownLocation.getTranslation().getData();
                    float pErrorY = DESIRED_MM_RED_NEAR_Y - xyzTranslation[1];

                    double yVector = 0.0f;
                    if (pErrorY < -25f) {
                        //yVector = -0.15f;
                        //strafeRight(0.5f);
                        strafeRightSlow();
                        // FIXME: "Slower" methods cannot use timer within them
                        //strafeRightSlower();
                    } else if (pErrorY > 25f) {
                        //yVector = 0.15f;
                        //strafeLeft(0.5f);
                        strafeLeftSlow();
                        // FIXME: cannot use slower method until timer pulled out of method
                        //strafeLeftSlower();
                    }
                    else {
                        //stopMoving();
                    }

                    if (Math.abs(pErrorY) <= 25f) {
                        stopMoving();

                        ++step;
                    }
                }
                else {
                    getTelemetryUtil().addData("Location:", "unknown");
                    stopMoving();
                }
                break;
            case 3://move toward first beacon
                forward(-0.13d);
                //if (getTimer().targetReached(1.5d)) {
                //if (getTimer().targetReached(0.75d)) {
                if (ultrasonicCache[0] < 10 || ultrasonicCache[0] > 150) {//back up until it is ten centimeters away from wall
                    stopMoving();
                    ++step;
                }
                break;
            case 4://press beacon
                forward(-0.2d);
                //if (getTimer().targetReached(0.3d)) {
                if (getTimer().targetReached(0.3d)) {
                    if (!firstShotComplete) {
                        shooter.setPower(0.95d);
                    }
                    stopMoving();
                    ++step;
                }
                break;
            case 5://move away from beacon
                forward(0.2d);
                //if (getTimer().targetReached(1.5d)) {
                //if (getTimer().targetReached(0.75d)) {
                if (ultrasonicCache[0] > 10f) {//back up until it is ten centimeters away from wall
                    stopMoving();
                    ++step;
                }
                break;
            case 6:
                if (getTimer().targetReached(1.4d)) {
                    beaconIsRed = colorSensor.red() > colorSensor.blue();
                    getTelemetryUtil().addData("red:", " " + colorSensor.red());
                    getTelemetryUtil().addData("blue:", " " + colorSensor.blue());
                    ++step;
                }
                break;
            case 7:
                if (!firstShotComplete) {
                    topCollector.setPower(0.5d);
                    midCollector.setPower(0.5d);
                    if (getTimer().targetReached(0.6d)) {
                        firstShotComplete = true;
                        topCollector.setPower(0.0d);
                        midCollector.setPower(0.0d);
                        shooter.setPower(0.0d);
                        ++step;
                        //step = 99999;
                    }
                }
                else {
                    ++step;
                }
                break;
            case 8:
                if(beaconIsRed)
                {
                    ++step;
                    //step = 20;
                }
                else
                {
                    if(getTimer().targetReached(2.2))
                    {
                        step = 3;
                        missedBeacon = true;
                    }
                }
                break;
            case 9://move away from beacon
                forward(0.2d);
                //if (getTimer().targetReached(1.5d)) {
                //if (getTimer().targetReached(0.75d)) {
                if (ultrasonicCache[0] > 20) {//back up until it is ten centimeters away from wall
                    stopMoving();
                    ++step;
                }
                break;
            case 10:
                strafeLeft(1.0d);
                if (getTimer().targetReached(1.8d)) {
                    stopMoving();
                    ++step;
                }
                break;
            case 11:
                stopMoving();
                // Ensure there isn't a stale location
                lastKnownLocation = null;
                if (getTimer().targetReached(0.25d)) {
                    step = 25;
                }
                break;
            case 12:
                isVisible = ((VuforiaTrackableDefaultListener)tools.getListener()).isVisible();
                robotLocationTransform = ((VuforiaTrackableDefaultListener)tools.getListener()).getUpdatedRobotLocation();
                getTelemetryUtil().addData("Target", "Looking for target.");
                if (robotLocationTransform != null) {
                    lastKnownLocation = robotLocationTransform;
                }

                if (lastKnownLocation != null && isVisible) {
                    getTelemetryUtil().addData("Location:", lastKnownLocation.formatAsTransform());
                    float[] xyzTranslation = lastKnownLocation.getTranslation().getData();
                    float pErrorY = DESIRED_MM_RED_FAR_Y - xyzTranslation[1];

                    double yVector = 0.0f;
                    if (pErrorY < -20f) {
                        //yVector = -0.15f;
                        //strafeRightSlow();
                        strafeRight(0.5);
                    } else if (pErrorY > 20f) {
                        //yVector = 0.15f;
                        //strafeLeftSlow();
                        strafeLeft(0.5);
                    }
                    else {
                        // This should get caught in clause below
                        //stopMoving();
                    }

                    if (Math.abs(pErrorY) <= 20f) {
                        stopMoving();

                        if (getRuntime() > 22.0d) {
                            step = 19;
                        }
                        else {
                            ++step;
                        }
                    }
                }
                else {
                    getTelemetryUtil().addData("Location:", "unknown");
                    //stopMoving();
                    //strafeLeftSlow();
                    //strafeLeft(1.0);
                    strafeLeft(.5);
                }
                break;
            case 13:
                isVisible = ((VuforiaTrackableDefaultListener)tools.getListener()).isVisible();
                robotLocationTransform = ((VuforiaTrackableDefaultListener)tools.getListener()).getUpdatedRobotLocation();
                getTelemetryUtil().addData("Target", "Looking for target.");
                if (robotLocationTransform != null) {
                    lastKnownLocation = robotLocationTransform;
                }

                if (lastKnownLocation != null && isVisible) {
                    Orientation orientation = Orientation.getOrientation(lastKnownLocation,
                            AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    float pErrorDegZ = DESIRED_DEGREES_RED_Z - orientation.thirdAngle;

                    double zVector = 0.0f;
                    if (pErrorDegZ < -17f) {
                        zVector = -0.15f;
                    } else if (pErrorDegZ > 17f) {
                        zVector = 0.15f;
                    }

                    turnLeft(zVector, true);

                    if (Math.abs(pErrorDegZ) <= 17f) {
                        stopMoving();

                        ++step;
                    }
                }
                else {
                    //turnLeft(0.12f, true);
                    turnLeft(0.17f, true);
                }

                break;
            case 14://move toward first beacon
                forward(-0.13d);
                //if (getTimer().targetReached(1.5d)) {
                //if (getTimer().targetReached(0.75d)) {
                if (ultrasonicCache[0] < 10 || ultrasonicCache[0] > 150) {//back up until it is ten centimeters away from wall
                    stopMoving();
                    ++step;
                }
                break;
            case 15://move away from beacon
                forward(-0.2d);
                //if (getTimer().targetReached(0.3d)) {
                if (getTimer().targetReached(0.3d)) {
                    stopMoving();
                    ++step;
                }
                break;
            case 16://move away from beacon
                forward(0.2d);
                //if (getTimer().targetReached(1.5d)) {
                //if (getTimer().targetReached(0.75d)) {
                if (ultrasonicCache[0] > 10) {//back up until it is ten centimeters away from wall
                    stopMoving();
                    ++step;
                }
                break;
            case 17:
                if (getTimer().targetReached(1.5)) {
                    beaconIsRed = colorSensor.red() > colorSensor.blue();
                    getTelemetryUtil().addData("red:", " " + colorSensor.red());
                    getTelemetryUtil().addData("blue:", " " + colorSensor.blue());
                    ++step;
                }
                break;
            case 18:
                if(beaconIsRed)
                {
                    ++step;
                    //step = 20;
                }
                else
                {
                    if(getTimer().targetReached(2.2))
                    {
                        step = 14;
                        missedBeacon = true;
                    }
                }
                break;
            case 19:  // Turn to drive to center vortex and ball
                step = 9999;
//                turnRight(0.5d, true);
//                if (getTimer().targetReached(0.31)) {
//                    stopMoving();
//                    ++step;
//                }
                break;
            case 20:
                forward(1.0d);
                if (getTimer().targetReached(1.15) ) {
                    stopMoving();
                    ++step;
                }
                break;
            case 25:
                isVisible = ((VuforiaTrackableDefaultListener)tools.getListener()).isVisible();
                robotLocationTransform = ((VuforiaTrackableDefaultListener)tools.getListener()).getUpdatedRobotLocation();
                getTelemetryUtil().addData("Target", "Looking for target.");
                if (robotLocationTransform != null) {
                    lastKnownLocation = robotLocationTransform;
                }

                if (lastKnownLocation != null && isVisible) {
                    getTelemetryUtil().addData("Location:", lastKnownLocation.formatAsTransform());
                    ++step;
                }
                else {
                    getTelemetryUtil().addData("Location:", "unknown");
                    //stopMoving();
                    strafeLeftSlow();
                    //strafeRight(0.5);
                }
                break;
            case 26:
                isVisible = ((VuforiaTrackableDefaultListener)tools.getListener()).isVisible();
                robotLocationTransform = ((VuforiaTrackableDefaultListener)tools.getListener()).getUpdatedRobotLocation();
                getTelemetryUtil().addData("Target", "Looking for target.");
                if (robotLocationTransform != null) {
                    lastKnownLocation = robotLocationTransform;
                }

                if (lastKnownLocation != null && isVisible) {
                    Orientation orientation = Orientation.getOrientation(lastKnownLocation,
                            AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    float pErrorDegZ = DESIRED_DEGREES_RED_Z - orientation.thirdAngle;

                    double zVector = 0.0f;
                    if (pErrorDegZ < -3f) {
                        zVector = -0.15f;
                    } else if (pErrorDegZ > 3f) {
                        zVector = 0.15f;
                    }

                    turnLeft(zVector, true);

                    if (Math.abs(pErrorDegZ) <= 3f) {
                        stopMoving();

                        ++step;
                    }
                }
                else {
                    //turnLeft(0.12f, true);
                    turnLeft(0.15f, true);
                }

                break;
            case 27:
                isVisible = ((VuforiaTrackableDefaultListener)tools.getListener()).isVisible();
                robotLocationTransform = ((VuforiaTrackableDefaultListener)tools.getListener()).getUpdatedRobotLocation();
                getTelemetryUtil().addData("Target", "Looking for target.");
                if (robotLocationTransform != null) {
                    lastKnownLocation = robotLocationTransform;
                }

                if (lastKnownLocation != null && isVisible) {
                    getTelemetryUtil().addData("Location:", lastKnownLocation.formatAsTransform());
                    float[] xyzTranslation = lastKnownLocation.getTranslation().getData();
                    float pErrorY = DESIRED_MM_RED_FAR_Y - xyzTranslation[1];

                    double yVector = 0.0f;
                    if (pErrorY < -20f) {
                        //yVector = -0.15f;
                        strafeRightSlow();
                        //strafeRight(0.5);
                    } else if (pErrorY > 20f) {
                        //yVector = 0.15f;
                        strafeLeftSlow();
                        //strafeLeft(0.5);
                    }
                    else {
                        // This should get caught in clause below
                        //stopMoving();
                    }

                    if (Math.abs(pErrorY) <= 20f) {
                        stopMoving();

                        if (getRuntime() > 22.0d) {
                            step = 19;
                        }
                        else {
                            step = 14;
                        }
                    }
                }
                else {
                    getTelemetryUtil().addData("Location:", "unknown");
                    //stopMoving();
                    strafeLeftSlow();
                    //strafeLeft(0.5);
                }
                break;
            case 999999:
                getTelemetryUtil().addData("distance", ultrasonicCache[0]);
                getTelemetryUtil().addData("play", "give up");
                break;
            default:
                stopMoving();
                break;
        }

        getTelemetryUtil().addData("distance", ultrasonicCache[0]);
        getTelemetryUtil().addData("Seen: ", isVisible ? "Visible" : "Not Visible");
        getTelemetryUtil().addData("runtime: ", "" + getRuntime());
        getTelemetryUtil().sendTelemetry();
    }
    public void forward (double power){
        frontRight.setPower(power);
        frontLeft.setPower(power);
        backRight.setPower(power);
        backLeft.setPower(power);
    }

    public void turnLeft (double power, boolean turnOnSpot){
        frontRight.setPower(power);
        backRight.setPower(power);
        if (turnOnSpot){
            backLeft.setPower(-power);
            frontLeft.setPower(-power);
        }
    }

    public void turnRight (double power, boolean turnOnSpot){
        frontLeft.setPower(power);
        backLeft.setPower(power);
        if (turnOnSpot){
            backRight.setPower(-power);
            frontRight.setPower(-power);
        }
    }
    public void strafeLeft (double power) {

        frontRight.setPower(1.0*power);
        backRight.setPower(1.0*-power);
        frontLeft.setPower(0.825*-power);
        backLeft.setPower(0.65*power);
    }
    public void strafeRight (double power) {

        frontRight.setPower(0.65*(-power));
        backRight.setPower(0.875*power);
        frontLeft.setPower(1.0*power);
        backLeft.setPower(0.75*(-power));
    }
    public void strafeLeftSlow() {
        double pow = .65;
        frontRight.setPower(0.475*pow);
        backRight.setPower(-0.5*pow);
        frontLeft.setPower(-0.45*pow);
        backLeft.setPower(0.45*pow);
    }
    public void strafeRightSlow() {
        double pow = .65;
        frontRight.setPower(-0.425*pow);
        backRight.setPower(0.5*pow);
        frontLeft.setPower(0.5*pow);
        backLeft.setPower(-0.5*pow);
    }


    public void stopMoving(){
        frontLeft.setPower(0.0d);
        frontRight.setPower(0.0d);
        backLeft.setPower(0.0d);
        backRight.setPower(0.0d);
    }
    public void strafeRightSlower() {
        double power = 0.8;
        double pow = .5;
        if(getTimer().targetReached(0.2)){
            frontRight.setPower(0.775*(-power));
            backRight.setPower(0.95*power);
            frontLeft.setPower(1.0*power);
            backLeft.setPower(0.75*(-power));
        }

         frontRight.setPower(-0.425*pow);
         backRight.setPower(0.5*pow);
         frontLeft.setPower(0.5*pow);
         backLeft.setPower(-0.5*pow);
        }

    public void strafeLeftSlower() {
        double power = 0.8;
        double pow = .5;
        if(getTimer().targetReached(0.2)){
            frontRight.setPower(0.925*power);
            backRight.setPower(1.0*-power);
            frontLeft.setPower(1.0*-power);
            backLeft.setPower(0.675*power);

        }

        frontRight.setPower(-0.425*pow);
        backRight.setPower(0.5*pow);
        frontLeft.setPower(0.5*pow);
        backLeft.setPower(-0.5*pow);
    }
}
