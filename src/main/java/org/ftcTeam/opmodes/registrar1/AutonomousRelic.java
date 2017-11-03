package org.ftcTeam.opmodes.registrar1;

import android.graphics.Bitmap;
import android.hardware.camera2.CameraDevice;
import android.util.Base64;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;
import android.graphics.Color;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import org.ftcbootstrap.ActiveOpMode;

import java.io.UnsupportedEncodingException;
import java.nio.ByteBuffer;
import java.util.ArrayList;


@Autonomous
public class AutonomousRelic extends ActiveOpMode {

    //License key
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

    // Vuforia init
    private VuforiaTrackables relicTrackables;
    private VuforiaTrackable relicTemplate;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables ftc2017Trackables;
    private ByteBuffer imgBuffer;
    private int byte1;
    private int byte2;
    private int byte3;
    private int byte0;
    private static String convert(byte[] thing) {
        try {
            return new String(thing, "US-ASCII");
        }
        catch (UnsupportedEncodingException uee) {
            return "Could not convert!!!";
        }
    }

    public static Image getImageFromFrame(VuforiaLocalizer.CloseableFrame frame, int format) {

        long numImgs = frame.getNumImages();
        for (int i = 0; i < numImgs; i++) {
            if (frame.getImage(i).getFormat() == format) {
                return frame.getImage(i);
            }//if
        }//for

        return null;
    }

    @Override
    protected void onInit() {
        getTelemetryUtil().addData("Init", getClass().getSimpleName() + " onInit."); //Show the init status
        getTelemetryUtil().sendTelemetry(); //Push update
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()); //camera id
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId); //init parameters
        parameters.vuforiaLicenseKey = KEY; //set license key
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; //set camera direction (front or back)
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters); //create vuforia object
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark"); //load the sample image to figure out
        relicTrackables.activate(); //activate it
        relicTemplate = relicTrackables.get(0); //fetch the first image
        vuforia.setFrameQueueCapacity(1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

    }
    /*
    private ArrayList<Double> RGBtoHSV(int r, int g, int b) {
        ArrayList<Double> vals = new ArrayList<Double>();
        double red = r / 255.0;
        double green = g / 255.0;
        double blue = b / 255.0;
        double cmax= Math.max(Math.max(red, green), blue);
        double cmin= Math.min(Math.min(red, green), blue);
        double luminance = (cmax+cmin)/2;
        double hue;
        double saturation;

        if (cmax == cmin) {
            hue = 0;
            saturation = 0;
        } else {
            double delta = cmax-cmin;
            if (luminance < 0.5) {
                saturation = delta / (cmax + cmin);
            } else {
                saturation = delta / (2 - cmax - cmin);
            }
            if (red == cmax) {
                hue = (green - blue) / delta;
            } else if (green == cmax) {
                hue = 2 + (blue - red) / delta;
            } else {
                hue = 4 + (red - green) / delta;
            }
            hue /= 6;
            if (hue < 0) {
                hue += 1;
            } else if (hue > 1) {
                hue -= 1;
            }
        }
        vals.add(hue);
        vals.add(luminance);
        vals.add(saturation);
        return vals;
    }*/

    public void RGBtoHSLEclipse(int r,int g,int b, double[] hsl) {
        double red = r / 255.0;
        double green = g / 255.0;
        double blue = b / 255.0;
        double cmax= Math.max(Math.max(red, green), blue);
        double cmin= Math.min(Math.min(red, green), blue);
        double  luminance = (cmax+cmin)/2;
        double hue = 0;
        double saturation = 0;
        if (cmax == cmin) {
            hue = 0;
            saturation = 0;
        } else {
            double delta = cmax-cmin;
           /* if (luminance < 0.5) {
                saturation = delta / (cmax + cmin);
            } else {
                saturation = delta / (2 - cmax - cmin);
            }*/
            saturation = delta / (1f - Math.abs(2f * luminance - 1f));
            if (red == cmax) {
                hue = (green - blue) / delta;
            } else if (green == cmax) {
                hue = 2 + (blue - red) / delta;
            } else {
                hue = 4 + (red - green) / delta;
            }
            hue /= 6;
            if (hue < 0) {
                hue += 1;
            } else if (hue > 1) {
                hue -= 1;
            }
        }

        hsl[0] = hue;
        hsl[1] = saturation;
        hsl[2] = luminance;

    }
    public void RGBToHSL(int r, int g, int b, float[] hsl) {
        final float rf = r / 255f;
        final float gf = g / 255f;
        final float bf = b / 255f;

        final float max = Math.max(rf, Math.max(gf, bf));
        final float min = Math.min(rf, Math.min(gf, bf));
        final float deltaMaxMin = max - min;

        float h, s;
        float l = (max + min) / 2f;

        if (max == min) {
            // Monochromatic
            h = s = 0f;
        } else {
            if (max == rf) {
                h = ((gf - bf) / deltaMaxMin) % 6f;
            } else if (max == gf) {
                h = ((bf - rf) / deltaMaxMin) + 2f;
            } else {
                h = ((rf - gf) / deltaMaxMin) + 4f;
            }

            s = deltaMaxMin / (1f - Math.abs(2f * l - 1f));
        }

        hsl[0] = (h * 60f) % 360f;
        hsl[1] = s;
        hsl[2] = l;
    }

    private String getColor(double hue, double sat, double lum) {
        String ret = "Unknown";
        if (hue <= 320 || hue <= 3) {
            if (lum >= .20 && lum <= .70) {
                if (sat >= .40) {
                    ret = "Red";
                    return ret;
                }
            }
        }
       /* if (hue <= 5 || hue >= 360) {
            if (lum >= .40) {
                if (sat <= 40) {
                    ret = "Red";
                }
            }
        }
       /* if (hue >= 320 || hue <= 10) {
            if (lum >= 20 && lum <= 70) {
              //  if (sat >= 40) {
                    ret = "Red";
              //  }
            }
        }*/
        if (hue >= 220 && hue <= 245) {
            if (lum >= 20 && lum <= 70) {
               // if (sat >= 40) {
                    ret = "Blue";
                //}
            }
        }
        return ret;
    }

    private void CheckImage(Image img) {
        Bitmap bm = Bitmap.createBitmap(img.getWidth(),img.getHeight(),Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(img.getPixels());
        for (int y = 0; y < bm.getHeight(); y++){
            for (int x = 0; x < bm.getWidth(); x++){
                int pix = bm.getPixel(x,y);
                int red = Color.red(pix);
                int green = Color.green(pix);
                int blue = Color.blue(pix);

                float[] hsv = new float[3];
                Color.RGBToHSV(red,green,blue,hsv);
                //Color.colorToHSV(Color.rgb(red,green,blue), hsv);
              //  getTelemetryUtil().addData("Stats", hsv[0] + " | " + hsv[1] + " | " + hsv[2]);
                if ((hsv[0] >= 0 && hsv[0] <= 10) || (hsv[0] >= 350)) {// h
                    if (hsv[1] >= .50) {//s
                        if (hsv[2] >= .70) {
                            getTelemetryUtil().addData("Red", "Yes. ");
                        }
                    }
                }
               // double[] hsl = new double[3];

                //RGBtoHSLEclipse(Color.red(pix),Color.green(pix),Color.blue(pix),hsl);

                //getTelemetryUtil().addData("Sat Lum", hsl[1] + " | " + hsl[2]);
               /* if (getColor(hsl[0], hsl[1], hsl[2]) != "Unknown") {
                   // getTelemetryUtil().addData("Color" + getColor(hsl[0], hsl[1], hsl[2]), "Found");
                }*/
              //  getTelemetryUtil().addData("PixelDat", "Red " + red + " Green " + green + " Blue " + blue);
                getTelemetryUtil().sendTelemetry();
            }
        } // gets the array of pixels.

        // convert to rgb888



    }
    private boolean doThing = true;
    @Override
    protected void activeLoop() throws InterruptedException {
        try {
            long millis1 = System.currentTimeMillis();
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (doThing) {
                doThing = false;
                CheckImage(getImageFromFrame(vuforia.getFrameQueue().take(), PIXEL_FORMAT.RGB565));
            }
            //getTelemetryUtil().addData("Image format before", vuforia.getFrameQueue().take().getImage(0).getFormat());
         /*   if (imgBuffer == null) {
                imgBuffer = vuforia.getFrameQueue().take().getImage(0).getPixels().duplicate();
                vuforia.setFrameQueueCapacity(0);
            }
            /*
            while (imgBuffer.position() < imgBuffer.limit()) {
                byte0 = imgBuffer.get() & 0xFF;
                byte1 = imgBuffer.get() & 0xFF;
                byte2 = imgBuffer.get() & 0xFF;
                byte3 = imgBuffer.get() & 0xFF;

                //  getTelemetryUtil().addData("Size", "" + imgBuffer.limit()); //bitmask
                //getTelemetryUtil().addData("Position", "" + imgBuffer.position());
                /*getTelemetryUtil().addData("[0] Alpha", "" + byte0);
                getTelemetryUtil().addData("[1] Red", "" + byte1);
                getTelemetryUtil().addData("[2] Green", "" + byte2);
                getTelemetryUtil().addData("[3] Blue", "" + byte3);
                double[] hsl = new double[3];
//               ./ float[] hsl = new float[3];
               //RGBtoHSL(byte1, byte2, byte3, hsl);
                RGBtoHSLEclipse(byte1, byte2, byte3, hsl);
                /*if (done == false) {
                    done = true;
                    getTelemetryUtil().addData("Data", "H" + HSV.get(0) + " L" + HSV.get(1) + " S" + HSV.get(2));
                }
                if (hsl[0] < 0) {
                    getTelemetryUtil().addData("ALERT < 0", "Hue val was " + hsl[0] + " from the vanilla byte " + byte1);
                }
              //  getTelemetryUtil().addData("Data", "Hue " + hsl[0] * 360 + " Saturation " + hsl[1] * 100 + " Luminance " + hsl[2] * 100);
                getTelemetryUtil().addData("RGB", "R" + byte1 + " G" + byte2 + " B" + byte3);
                //String ye = getColor(hsl[0] * 360, hsl[1] * 100, hsl[2] * 100);
                if (ye.equalsIgnoreCase("red")) {
                    getTelemetryUtil().addData("Found1", ye);
                }
                if (ye.equalsIgnoreCase("blue")) {
                    getTelemetryUtil().addData("Found2", ye);
                }
                getTelemetryUtil().sendTelemetry();
            }*/
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                // Image img =  vuforia.getFrameQueue().take().getImage(0); //format 8888
                getTelemetryUtil().addData("Target", vuMark.name());
                getTelemetryUtil().addData("Elapsed time", System.currentTimeMillis() - millis1 + "ms");
                //getTelemetryUtil().addData("Dimensions", img.getHeight() + "h " + img.getWidth() + "w");
            }
        }
        catch (Throwable t) {
            getTelemetryUtil().addData("Throwable: ", t.getMessage());
        }
        finally {
            getTelemetryUtil().sendTelemetry();
        }
    }
}
