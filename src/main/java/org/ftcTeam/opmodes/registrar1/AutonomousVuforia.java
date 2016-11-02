package org.ftcTeam.opmodes.registrar1;


import android.util.Base64;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.Vuforia;

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
import org.ftcbootstrap.components.operations.motors.GamePadTankDrive;

import com.qualcomm.ftcrobotcontroller.R;

import java.io.UnsupportedEncodingException;

/**
 * Note:  It is assumed that the proper registry is used for this set of demos. To confirm please
 * search for "Enter your custom registry here"  in  {@link org.ftcTeam.FTCTeamControllerActivity}
 * <p/>
 * Summary:  Use an Operation class to perform a tank drive using the gamepad joysticks.
 * See: {@link GamePadTankDrive}
 */

@Autonomous
public class AutonomousVuforia extends ActiveOpMode {
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

    private static final String convert(byte[] thing) {
        try {
            return new String(thing, "US-ASCII");
        }
        catch (UnsupportedEncodingException uee) {
            return "Could not convert!!!";
        }
    }

    public static final float MM_PER_INCH = 25.4f;
    public static final float MM_BOT_WIDTH = 18 * MM_PER_INCH;            // ... or whatever is right for your robot
    public static final float MM_FTC_FIELD_WIDTH = (12*12 - 2) * MM_PER_INCH;   // the FTC field is ~11'10" center-to-center of the glass panels
    public static final float MM_TARGET_CENTER_HEIGHT = (1.5f + (8.5f / 2.0f)) * MM_PER_INCH; // half height of 8.5x11 sheet plus 1.5" above floor
    public static final float MM_NEAR_OFFSET = (1 * 12) * MM_PER_INCH;
    public static final float MM_FAR_OFFSET = (3 * 12) * MM_PER_INCH;

    VuforiaLocalizer.Parameters parameters;
    VuforiaLocalizer vuforia;
    VuforiaTrackables ftc2016Trackables;

    /**
     * Implement this method to define the code to run when the Init button is pressed on the Driver station.
     */
    @Override
    protected void onInit() {
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        ftc2016Trackables = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");

        VuforiaTrackable wheels = ftc2016Trackables.get(0);
        VuforiaTrackable tools  = ftc2016Trackables.get(1);
        VuforiaTrackable legos  = ftc2016Trackables.get(2);
        VuforiaTrackable gears  = ftc2016Trackables.get(3);

        wheels.setName("wheels"); // blue-center
        tools.setName("tools"); // red-far
        legos.setName("legos"); // blue-far
        gears.setName("gears"); // red-center

        OpenGLMatrix redNearTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                // This also translates +z to be correct height above floor
                // This translates -y to move along wall toward Red team
                .translation(-MM_FTC_FIELD_WIDTH/2, -MM_NEAR_OFFSET, MM_TARGET_CENTER_HEIGHT)
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
                .translation(-MM_FTC_FIELD_WIDTH/2, MM_FAR_OFFSET, MM_TARGET_CENTER_HEIGHT)
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
                .translation(MM_NEAR_OFFSET, MM_FTC_FIELD_WIDTH/2, MM_TARGET_CENTER_HEIGHT)
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
                .translation(-MM_FAR_OFFSET, MM_FTC_FIELD_WIDTH/2, MM_TARGET_CENTER_HEIGHT)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        legos.setLocation(blueFarTargetLocationOnField);

        // FIXME: +Y translation puts it on front of robot, and -Y to put on back
        // FIXME: Would need -90 rotation on X axis to put it facing inwards on back
        // FIXME: Still need +z translation to put at proper height
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(0, MM_BOT_WIDTH/2, 0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));

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

    }

}
