package org.firstinspires.ftc.teamcode.demos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class ColorSensorBlinkin {
    public static int full = 78;
    public static int empty = 99;
    public static int aThresh = 300;
    public ElapsedTime patTime = null;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern purp, gren, yelo, whit, non, bluo;

    PixelClassifier classifier1 = null;
    PixelClassifier classifier2 = null;
    RevColorSensorV3 colorSensor, colorSensor2;
    public boolean pixelPattern = true;
    int lastPixel1 = 0;
    int lastPixel2 = 0;
    int pixel1 = 0; // 0: non, 1: white, 2: yellow, 3: green, 4: purple
    int pixel2 = 0; // 0: non, 1: white, 2: yellow, 3: green, 4: purple
    int pix1Now = 0;
    int pix2Now = 0;
    public ColorSensorBlinkin(String blinkinName, String colorSensor1Name, String colorSensor2Name, HardwareMap hardwareMap) {
        // go [blue 0.8] when 2 pixels are in
        // then repeat [front color 0.4] [off 0.2] [back color 0.4] [off 0.5]
         colorSensor = hardwareMap.get(RevColorSensorV3.class, colorSensor1Name);
         colorSensor2 = hardwareMap.get(RevColorSensorV3.class, colorSensor2Name);
        patTime = new ElapsedTime();
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        purp = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
        gren = RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN;
        yelo = RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE;
        whit = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        non = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        bluo = RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
        classifier1 = new PixelClassifier();
        classifier2 = new PixelClassifier();
        classifier1.addClassifier(0, new int[]{85,145,176,135});
        classifier1.addClassifier(1, new int[]{2370,4198,3679,3414});
        classifier1.addClassifier(2, new int[]{1316,1892,525,1244});
        classifier1.addClassifier(3, new int[]{401,1282,525,735});
        classifier1.addClassifier(4, new int[]{1059,1563,2197,1607});

        classifier2.addClassifier(0, new int[]{71,140,212,139});
        classifier2.addClassifier(1, new int[]{1600,3241,3450,2686});
        classifier2.addClassifier(2, new int[]{808,1320,415,851});
        classifier2.addClassifier(3, new int[]{268,970,455,565});
        classifier2.addClassifier(4, new int[]{695,1170,2001,1289});

    }
    public double[][] tick() {
        non = RevBlinkinLedDriver.BlinkinPattern.fromNumber(empty);
        int red1 = colorSensor.red();
        int green1 = colorSensor.green();
        int blue1 = colorSensor.blue();
        int alpha1 = colorSensor.alpha();

        int red2 = colorSensor2.red();
        int green2 = colorSensor2.green();
        int blue2 = colorSensor2.blue();
        int alpha2 = colorSensor2.alpha();


        pixel1 = classifier1.classify(new int[]{red1, green1, blue1, alpha1});
        if (pixel1 == -1) pixel1 = lastPixel1;
        pixel2 = classifier2.classify(new int[]{red2, green2, blue2, alpha2});
        if (pixel2 == -1) pixel2 = lastPixel2;

        if (pixelPattern){
            if (pix1Now == 0 && pix2Now == 0) blinkinLedDriver.setPattern(non);
            else if (pix1Now != 0 && pix2Now != 0){
                if (patTime.milliseconds() > 1200) pixelPattern = false;
                if (patTime.milliseconds() < 500) blinkinLedDriver.setPattern(pix2Now == 1 ? whit : (pix2Now == 2 ? yelo : (pix2Now == 3 ? gren : purp)));
                else if (patTime.milliseconds() < 700) blinkinLedDriver.setPattern(non);
                else if (patTime.milliseconds() < 1200) blinkinLedDriver.setPattern(pix1Now == 1 ? whit : (pix1Now == 2 ? yelo : (pix1Now == 3 ? gren : purp)));
            }
            else {
                if (patTime.milliseconds() > 500) pixelPattern = false;
                if (pix1Now != 0) blinkinLedDriver.setPattern(pix1Now == 1 ? whit : (pix1Now == 2 ? yelo : (pix1Now == 3 ? gren : purp)));
                else blinkinLedDriver.setPattern(pix2Now == 1 ? whit : (pix2Now == 2 ? yelo : (pix2Now == 3 ? gren : purp)));
            }
        } else if ((lastPixel2 == 0 || lastPixel1 == 0) && (pixel2 != 0 && pixel1 != 0)){
            pixelPattern = true;
            patTime.reset();
            pix1Now = pixel1;
            pix2Now = pixel2;
        }
        else {
            blinkinLedDriver.setPattern(non);
        }
        lastPixel1 = pixel1;
        lastPixel2 = pixel2;
        return new double[][]{{red1, green1, blue1, alpha1},{red2, green2, blue2, alpha2}};

    }
    public int[] pixelBlink() {
        patTime.reset();
        pix1Now = pixel1;
        pix2Now = pixel2;
        pixelPattern = true;
        return new int[]{pixel1, pixel2};
    }
    public int[] getPixelStates() {
        return new int[]{pixel1, pixel2};
    }
    public void setBlinkin(RevBlinkinLedDriver.BlinkinPattern pattern) {
        pixelPattern = false;
        blinkinLedDriver.setPattern(pattern);
    }
    public void setPixelPattern(boolean on) {
        pixelPattern = on;
    }
}
