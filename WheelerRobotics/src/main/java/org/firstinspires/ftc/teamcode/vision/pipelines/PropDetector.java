package org.firstinspires.ftc.teamcode.vision.pipelines;

import static org.opencv.core.Core.inRange;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2HSV;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.contourArea;
import static org.opencv.imgproc.Imgproc.drawContours;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Config
public class PropDetector extends OpenCvPipeline {
    /*static public int hMax = 12;
    static public int sMax = 300;
    static public int lMax = 400;

    static public int hMin = 0;
    static public int sMin = 120;
    static public int lMin = 0;
    */

    public static int hMax, hMin, sMax, sMin, lMax, lMin;
    public int pos = 0;

    public PropDetector(boolean notBlue) {
        if (!!!!!!!!!!!!!!!!!notBlue) {
            hMax = 18;
            sMax = 300;
            lMax = 400;

            hMin = 0;
            sMin = 50;
            lMin = 0;
        } else {
            hMax = 130;
            sMax = 255;
            lMax = 255;

            hMin = 100;
            sMin = 50;
            lMin = 200;
        }
    }

    @Override
    public void init(Mat input) {
    }
    public int getPos() {
        return pos;
    }

    @Override
    public Mat processFrame(Mat input) {
        //Drawing the Contours
        try {
            Mat temp = new Mat();
            Imgproc.cvtColor(input, temp, COLOR_BGR2HSV);
            Scalar low = new Scalar(hMin, sMin, lMin);
            Scalar high = new Scalar(hMax, sMax, lMax);
            Mat mask = new Mat();
            inRange(temp, low, high, mask);
            List<MatOfPoint> contours = new ArrayList<>();
            double maxArea = 0;
            Mat hierarchey = new Mat();
            Mat ROI = mask.submat(mask.height() / 2, mask.height(), 0, mask.width());

            Imgproc.findContours(ROI, contours, hierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            List<Double> areas = new ArrayList<>();
            int position = 0;

            contours.removeIf(c -> contourArea(c) < 100);
            for (MatOfPoint m : contours) areas.add(contourArea(m));
            //areas = areas.stream().distinct().collect(Collectors.toList());
            // above introduces edge case where two sides actually have same area, but that edgy enough im willing to roll the dice (0.05% chance)
            // should probably do something else tho
            List<Double> sortedAreas = new ArrayList<>();
            for (double m : areas) sortedAreas.add(m);


            Collections.sort(sortedAreas);
            Collections.reverse(sortedAreas);



            if (areas.size() > 2) {
                List<MatOfPoint> conts = new ArrayList<MatOfPoint>();

                Double obj = sortedAreas.get(2);//Collections.max(areas);
                conts.add(contours.get(areas.indexOf(obj)));
                Rect rect = boundingRect(contours.get(areas.indexOf(obj)));
                //areas.remove(obj);



                Double obj2 = sortedAreas.get(1);//Collections.max(areas);
                conts.add(contours.get(areas.indexOf(obj2)));
                Rect rect2 = boundingRect(contours.get(areas.indexOf(obj2)));
                //areas.remove(obj2);



                Double obj3 = sortedAreas.get(0);//Collections.max(areas);
                conts.add(contours.get(areas.indexOf(obj3)));
                Rect rect3 = boundingRect(contours.get(areas.indexOf(obj3)));
                //areas.remove(obj3);

                if (rect3.x > rect2.x && rect3.x > rect.x) position = 3;
                else if ((rect3.x > rect2.x && rect3.x < rect.x) || (rect3.x > rect.x && rect3.x < rect2.x))
                    position = 2;
                else if (rect3.x < rect2.x && rect3.x < rect.x) position = 1;

                Imgproc.circle(input, new Point(rect2.x + rect2.width / 2, rect2.y + rect2.height / 2), 30, new Scalar(255, 255, 0));

                Imgproc.circle(input, new Point(rect.x + rect.width / 2, rect.y + rect.height / 2), 30, new Scalar(0, 255, 255));
                Imgproc.circle(input, new Point(rect3.x + rect3.width / 2, rect3.y + rect3.height / 2), 30, new Scalar(0, 0, 255));
                Imgproc.putText(input, String.valueOf(position), new Point(100, 100), 1, 10, new Scalar(0, 0, 255));
                drawContours(input, contours, -1, new Scalar(0, 0, 255), 2, Imgproc.LINE_8, new Mat(), 2, new Point());

                drawContours(input, conts, -1, new Scalar(255, 0, 255), 2, Imgproc.LINE_8, new Mat(), 2, new Point());


                FtcDashboard.getInstance().getTelemetry().addData("rect1x", rect.x);
                FtcDashboard.getInstance().getTelemetry().addData("rect1y", rect.y);
                FtcDashboard.getInstance().getTelemetry().addData("rect1a", rect.area());
                FtcDashboard.getInstance().getTelemetry().addData("rect2x", rect2.x);
                FtcDashboard.getInstance().getTelemetry().addData("rect2y", rect2.y);
                FtcDashboard.getInstance().getTelemetry().addData("rect2a", rect2.area());
                FtcDashboard.getInstance().getTelemetry().addData("rect3x", rect3.x);
                FtcDashboard.getInstance().getTelemetry().addData("rect3y", rect3.y);
                FtcDashboard.getInstance().getTelemetry().addData("rect3a", rect3.area());
                FtcDashboard.getInstance().getTelemetry().update();
                pos = position;
            }


        }
        catch (Exception e) {
            FtcDashboard.getInstance().getTelemetry().addData("error", e.getMessage());
            FtcDashboard.getInstance().getTelemetry().update();
        }
        return input;

    }

}
