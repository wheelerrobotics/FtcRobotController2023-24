package org.firstinspires.ftc.teamcode.vision.pipelines;

import static org.opencv.core.Core.inRange;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2HSV;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.contourArea;

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
    static public int hMax = 50;
    static public int sMax = 255;
    static public int lMax = 250;

    static public int hMin = 0;
    static public int sMin = 70;
    static public int lMin = 50;

    @Override
    public void init(Mat input) {
    }

    @Override
    public Mat processFrame(Mat input) {
        //Drawing the Contours
        Mat temp= new Mat();
        Imgproc.cvtColor(input,temp,COLOR_BGR2HSV);
        Scalar low= new Scalar(hMin,sMin,lMin);
        Scalar high= new Scalar(hMax,sMax,lMax);
        Mat mask = new Mat();
        inRange(temp,low,high,mask);
        List<MatOfPoint> contours = new ArrayList<>();
        double maxArea = 0;
        MatOfPoint cont = null;
        Mat hierarchey = new Mat();
        Imgproc.findContours(mask, contours, hierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        List<Double> areas = new ArrayList<>();
        int position = 0;

        contours.forEach((c) -> areas.add(contourArea(c)));

        if (areas.size() > 2) {

            Double obj = Collections.max(areas);
            areas.remove(obj);
            Double obj2 = Collections.max(areas);
            areas.remove(obj2);
            Double obj3 = Collections.max(areas);
            areas.remove(obj3);

            Rect cont1 = boundingRect(contours.get(areas.indexOf(obj)));
            Rect cont2 = boundingRect(contours.get(areas.indexOf(obj2)));
            Rect cont3 = boundingRect(contours.get(areas.indexOf(obj3)));

            if (cont1.x > cont2.x && cont1.x > cont3.x) position = 3;
            else if ((cont1.x > cont2.x && cont1.x < cont3.x) || (cont1.x > cont3.x && cont1.x < cont2.x)) position = 2;
            else if (cont1.x < cont2.x && cont1.x < cont3.x) position = 1;



            cont = contours.get(areas.indexOf(obj));
            Rect rect = boundingRect(cont);
            Imgproc.circle(input, new Point(rect.x + rect.width/2, rect.y + rect.height/2), 30, new Scalar(0, 255, 255));
            Imgproc.putText(input, String.valueOf(position), new Point(100, 100), 1, 10, new Scalar(0, 0, 255));

        }

        //drawContours(input, contours,-1, new Scalar(255, 0, 255), 2, Imgproc.LINE_8, hierarchey, 2, new Point() );

        return input;

    }

}
