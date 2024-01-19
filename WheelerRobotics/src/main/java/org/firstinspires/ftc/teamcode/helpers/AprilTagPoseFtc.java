package org.firstinspires.ftc.teamcode.helpers;

public class AprilTagPoseFtc
{

    /**
     * X translation of AprilTag, relative to camera lens.  Measured sideways (Horizontally in camera image) the positive X axis extends out to the right of the camera viewpoint. <BR>
     * An x value of zero implies that the Tag is centered between the left and right sides of the Camera image.
     */
    public double x;

    /**
     * Y translation of AprilTag, relative to camera lens.  Measured forwards (Horizontally in camera image) the positive Y axis extends out in the direction the camera is pointing.<BR>
     * A y value of zero implies that the Tag is touching (aligned with) the lens of the camera, which is physically unlikley.  This value should always be positive.  <BR>
     */
    public double y;

    /**
     * Z translation of AprilTag, relative to camera lens.  Measured upwards (Vertically in camera image) the positive Z axis extends Upwards in the camera viewpoint.<BR>
     * A z value of zero implies that the Tag is centered between the top and bottom of the camera image.
     */
    public double z;

    /**
     * Rotation of AprilTag around the Z axis.  Right-Hand-Rule defines positive Yaw rotation as Counter-Clockwise when viewed from above.<BR>
     * A yaw value of zero implies that the camera is directly in front of the Tag, as viewed from above.
     */
    public double yaw;

    /**
     * Rotation of AprilTag around the X axis.  Right-Hand-Rule defines positive Pitch rotation as the Tag Image face twisting down when viewed from the camera.<BR>
     * A pitch value of zero implies that the camera is directly in front of the Tag, as viewed from the side.
     */
    public double pitch;

    /**
     * Rotation of AprilTag around the Y axis.  Right-Hand-Rule defines positive Roll rotation as the Tag Image rotating Clockwise when viewed from the camera.<BR>
     * A roll value of zero implies that the Tag image is alligned squarely and upright, when viewed in the camera image frame.
     */
    public double roll;

    /**
     * Range, (Distance), from the Camera lens to the center of the Tag, as measured along the X-Y plane (across the ground).
     */
    public double range;

    /**
     * Bearing, or Horizontal Angle, from the "camera center-line", to the "line joining the Camera lens and the Center of the Tag".  <BR>
     * This angle is measured across the X-Y plane (across the ground).<BR>
     * A positive Bearing indicates that the robot must employ a positive Yaw (rotate counter clockwise) in order to point towards the target.
     */
    public double bearing;

    /**
     * Elevation, (Vertical Angle), from "the camera center-line", to "the line joining the Camera Lens and the Center of the Tag".<BR>
     * A positive Elevation indicates that the robot must employ a positive Pitch (tilt up) in order to point towards the target.
     */
    public double elevation;
}
