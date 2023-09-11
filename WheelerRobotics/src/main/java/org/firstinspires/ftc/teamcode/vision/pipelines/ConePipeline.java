package org.firstinspires.ftc.teamcode.vision.pipelines;

import android.os.Environment;

import org.opencv.core.Mat;
import org.opencv.core.MatOfRect;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.CascadeClassifier;
import org.openftc.easyopencv.OpenCvPipeline;

public class ConePipeline extends OpenCvPipeline {
    public CascadeClassifier faceDetector;

    public MatOfRect faceDetections;
    @Override
    public void init(Mat input) {
        faceDetector = new CascadeClassifier(String.format("%s/z/haarcascade_frontalface_default.xml", Environment.getExternalStorageDirectory().getAbsolutePath()));
    }

    @Override
    public Mat processFrame(Mat input) {
        faceDetections = new MatOfRect();
        faceDetector.detectMultiScale(input, faceDetections);
        // Creating a rectangular box showing faces detected
        for (Rect rect : faceDetections.toArray()) {
            Imgproc.circle(input, new Point(rect.x + rect.width/2, rect.y + rect.height/2),
                    rect.width/4 + rect.height/4, new Scalar(255, 0, 255));
        }

        return input;

    }

}
