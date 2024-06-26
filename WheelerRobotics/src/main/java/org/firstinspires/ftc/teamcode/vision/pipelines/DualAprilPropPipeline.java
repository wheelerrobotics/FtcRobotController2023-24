package org.firstinspires.ftc.teamcode.vision.pipelines;

import static org.opencv.core.Core.inRange;
import static org.opencv.imgproc.Imgproc.COLOR_BGR2HSV;
import static org.opencv.imgproc.Imgproc.boundingRect;
import static org.opencv.imgproc.Imgproc.contourArea;
import static org.opencv.imgproc.Imgproc.drawContours;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class DualAprilPropPipeline extends OpenCvPipeline {
    public static int propSize = 5000;
    public static int pixelSize = 600;
    public boolean notB = false;
    public static int xcutoff = 200;
    public boolean yellow = false;
    public static int yhMax = 115, yhMin = 95, ysMax = 300, ysMin = 100, ylMax = 400, ylMin = 0;
    public static int hMax, hMin, sMax, sMin, lMax, lMin;
    public int pos = 0;
    public boolean weBeProppin = true;

    public DualAprilPropPipeline(boolean notBlue, double tagsize, double fx, double fy, double cx, double cy) {
        this.tagsize = tagsize;
        this.tagsizeX = tagsize;
        this.tagsizeY = tagsize;
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
        this.notB = notBlue;
        constructMatrix();

        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, -1, 3);
        if (!!!!!!!!!!!!!!!!!notBlue) {
            hMax = 18;
            sMax = 300;
            lMax = 400;

            hMin = 0;
            sMin = 50;
            lMin = 0;
        } else {
            hMax = 125;
            sMax = 255;
            lMax = 255;

            hMin = 110;
            sMin = 50;// 50 bef
            lMin = 0; // 200 bef
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
        if (!notB) Core.flip(input, input, -1); //only for flipped cams
        //Core.flip(input, input, -1);
        //Drawing the Contours
        try {

            if (weBeProppin) {
            /*
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

                if (areas.size() == 1) {

                }
                if (areas.size() == 2) {

                }
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

                */
                Mat temp = new Mat();
                Imgproc.cvtColor(input, temp, COLOR_BGR2HSV);
                Scalar low = new Scalar(hMin, sMin, lMin);
                Scalar high = new Scalar(hMax, sMax, lMax);
                if (yellow) {
                    low = new Scalar(yhMin, ysMin, ylMin);
                    high = new Scalar(yhMax, ysMax, ylMax);
                }
                Mat mask = new Mat();
                inRange(temp, low, high, mask);
                List<MatOfPoint> contours = new ArrayList<>();
                double maxArea = 0;
                Mat hierarchey = new Mat();
                Mat ROI = yellow ? mask : mask.submat(mask.height() / 2, mask.height(), 0, mask.width());

                Imgproc.findContours(ROI, contours, hierarchey, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
                List<Double> areas = new ArrayList<>();
                int position = 0;

                if (yellow) contours.removeIf(c -> contourArea(c) < pixelSize);
                else contours.removeIf(c -> contourArea(c) < propSize);

                if (contours.size() == 1) {
                    Rect rect3 = boundingRect(contours.get(0));

                    if (rect3.x > xcutoff) pos = 3;
                    else pos = 2;
                    if (yellow) pos = 1;
                } if (contours.size() == 0) {
                    pos = 1;
                    if (yellow) pos = 0;
                }
                FtcDashboard.getInstance().getTelemetry().addData("pos", pos);
                FtcDashboard.getInstance().getTelemetry().update();
                drawContours(input, contours, -1, new Scalar(0, 0, 255), 2, Imgproc.LINE_8, new Mat(), 2, new Point());


            }else {
                // Convert to greyscale
                Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2GRAY);
                //Core.flip(grey, grey, 0);
                //Core.flip(input, input, 0);
                //grey = input;

                synchronized (decimationSync)
                {
                    if(needToSetDecimation)
                    {
                        AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeApriltagPtr, decimation);
                        needToSetDecimation = false;
                    }
                }

                // Run AprilTag
                detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy);

                synchronized (detectionsUpdateSync)
                {
                    detectionsUpdate = detections;
                }

                // For fun, use OpenCV to draw 6DOF markers on the image. We actually recompute the pose using
                // OpenCV because I haven't yet figured out how to re-use AprilTag's pose in OpenCV.
                FtcDashboard.getInstance().getTelemetry().addData("stufs", detections.size());
                for(AprilTagDetection detection : detections)
                {
                    Pose pose = poseFromTrapezoid(detection.corners, cameraMatrix, tagsizeX, tagsizeY);
                    drawAxisMarker(input, tagsizeY/2.0, 6, pose.rvec, pose.tvec, cameraMatrix);
                    draw3dCubeMarker(input, tagsizeX, tagsizeX, tagsizeY, 5, pose.rvec, pose.tvec, cameraMatrix);
                }

                return input;
            }


        }
        catch (Exception e) {
            FtcDashboard.getInstance().getTelemetry().addData("error", e.getMessage());
            FtcDashboard.getInstance().getTelemetry().update();
        }
        return input;

    }
    private long nativeApriltagPtr;
    private Mat grey = new Mat();
    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
    private final Object detectionsUpdateSync = new Object();

    Mat cameraMatrix;

    Scalar blue = new Scalar(7,197,235,255);
    Scalar red = new Scalar(255,0,0,255);
    Scalar green = new Scalar(0,255,0,255);
    Scalar white = new Scalar(255,255,255,255);

    public static double fx;
    public static double fy;
    public static double cx;
    public static double cy;

    // UNITS ARE METERS
    public static double tagsize;
    public static double tagsizeX;
    public static double tagsizeY;

    private float decimation;
    private boolean needToSetDecimation;
    private final Object decimationSync = new Object();

    @Override
    public void finalize()
    {
        // Might be null if createApriltagDetector() threw an exception
        if(nativeApriltagPtr != 0)
        {
            // Delete the native context we created in the constructor
            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
            nativeApriltagPtr = 0;
        }
        else
        {
            System.out.println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL");
        }
    }

    public void setDecimation(float decimation)
    {
        synchronized (decimationSync)
        {
            this.decimation = decimation;
            needToSetDecimation = true;
        }
    }

    public ArrayList<AprilTagDetection> getLatestDetections()
    {
        return detections;
    }

    public ArrayList<AprilTagDetection> getDetectionsUpdate()
    {
        synchronized (detectionsUpdateSync)
        {
            ArrayList<AprilTagDetection> ret = detectionsUpdate;
            detectionsUpdate = null;
            return ret;
        }
    }

    void constructMatrix()
    {
        //     Construct the camera matrix.
        //
        //      --         --
        //     | fx   0   cx |
        //     | 0    fy  cy |
        //     | 0    0   1  |
        //      --         --
        //

        cameraMatrix = new Mat(3,3, CvType.CV_32FC1);

        cameraMatrix.put(0,0, fx);
        cameraMatrix.put(0,1,0);
        cameraMatrix.put(0,2, cx);

        cameraMatrix.put(1,0,0);
        cameraMatrix.put(1,1,fy);
        cameraMatrix.put(1,2,cy);

        cameraMatrix.put(2, 0, 0);
        cameraMatrix.put(2,1,0);
        cameraMatrix.put(2,2,1);
    }

    /**
     * Draw a 3D axis marker on a detection. (Similar to what Vuforia does)
     *
     * @param buf the RGB buffer on which to draw the marker
     * @param length the length of each of the marker 'poles'
     * @param rvec the rotation vector of the detection
     * @param tvec the translation vector of the detection
     * @param cameraMatrix the camera matrix used when finding the detection
     */
    void drawAxisMarker(Mat buf, double length, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
    {
        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(0,0,0),
                new Point3(length,0,0),
                new Point3(0,length,0),
                new Point3(0,0,-length)
        );

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Draw the marker!
        Imgproc.line(buf, projectedPoints[0], projectedPoints[1], red, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[2], green, thickness);
        Imgproc.line(buf, projectedPoints[0], projectedPoints[3], blue, thickness);

        Imgproc.circle(buf, projectedPoints[0], thickness, white, -1);
    }

    void draw3dCubeMarker(Mat buf, double length, double tagWidth, double tagHeight, int thickness, Mat rvec, Mat tvec, Mat cameraMatrix)
    {
        //axis = np.float32([[0,0,0], [0,3,0], [3,3,0], [3,0,0],
        //       [0,0,-3],[0,3,-3],[3,3,-3],[3,0,-3] ])

        // The points in 3D space we wish to project onto the 2D image plane.
        // The origin of the coordinate space is assumed to be in the center of the detection.
        MatOfPoint3f axis = new MatOfPoint3f(
                new Point3(-tagWidth/2, tagHeight/2,0),
                new Point3( tagWidth/2, tagHeight/2,0),
                new Point3( tagWidth/2,-tagHeight/2,0),
                new Point3(-tagWidth/2,-tagHeight/2,0),
                new Point3(-tagWidth/2, tagHeight/2,-length),
                new Point3( tagWidth/2, tagHeight/2,-length),
                new Point3( tagWidth/2,-tagHeight/2,-length),
                new Point3(-tagWidth/2,-tagHeight/2,-length));

        // Project those points
        MatOfPoint2f matProjectedPoints = new MatOfPoint2f();
        Calib3d.projectPoints(axis, rvec, tvec, cameraMatrix, new MatOfDouble(), matProjectedPoints);
        Point[] projectedPoints = matProjectedPoints.toArray();

        // Pillars
        for(int i = 0; i < 4; i++)
        {
            Imgproc.line(buf, projectedPoints[i], projectedPoints[i+4], blue, thickness);
        }

        // Base lines
        //Imgproc.line(buf, projectedPoints[0], projectedPoints[1], blue, thickness);
        //Imgproc.line(buf, projectedPoints[1], projectedPoints[2], blue, thickness);
        //Imgproc.line(buf, projectedPoints[2], projectedPoints[3], blue, thickness);
        //Imgproc.line(buf, projectedPoints[3], projectedPoints[0], blue, thickness);

        // Top lines
        Imgproc.line(buf, projectedPoints[4], projectedPoints[5], green, thickness);
        Imgproc.line(buf, projectedPoints[5], projectedPoints[6], green, thickness);
        Imgproc.line(buf, projectedPoints[6], projectedPoints[7], green, thickness);
        Imgproc.line(buf, projectedPoints[4], projectedPoints[7], green, thickness);
    }

    /**
     * Extracts 6DOF pose from a trapezoid, using a camera intrinsics matrix and the
     * original size of the tag.
     *
     * @param points the points which form the trapezoid
     * @param cameraMatrix the camera intrinsics matrix
     * @param tagsizeX the original width of the tag
     * @param tagsizeY the original height of the tag
     * @return the 6DOF pose of the camera relative to the tag
     */
    Pose poseFromTrapezoid(Point[] points, Mat cameraMatrix, double tagsizeX , double tagsizeY)
    {
        // The actual 2d points of the tag detected in the image
        MatOfPoint2f points2d = new MatOfPoint2f(points);

        // The 3d points of the tag in an 'ideal projection'
        Point3[] arrayPoints3d = new Point3[4];
        arrayPoints3d[0] = new Point3(-tagsizeX/2, tagsizeY/2, 0);
        arrayPoints3d[1] = new Point3(tagsizeX/2, tagsizeY/2, 0);
        arrayPoints3d[2] = new Point3(tagsizeX/2, -tagsizeY/2, 0);
        arrayPoints3d[3] = new Point3(-tagsizeX/2, -tagsizeY/2, 0);
        MatOfPoint3f points3d = new MatOfPoint3f(arrayPoints3d);

        // Using this information, actually solve for pose
        Pose pose = new Pose();
        Calib3d.solvePnP(points3d, points2d, cameraMatrix, new MatOfDouble(), pose.rvec, pose.tvec, false);

        return pose;
    }

    /*
     * A simple container to hold both rotation and translation
     * vectors, which together form a 6DOF pose.
     */
    class Pose
    {
        Mat rvec;
        Mat tvec;

        public Pose()
        {
            rvec = new Mat();
            tvec = new Mat();
        }

        public Pose(Mat rvec, Mat tvec)
        {
            this.rvec = rvec;
            this.tvec = tvec;
        }
    }
}
