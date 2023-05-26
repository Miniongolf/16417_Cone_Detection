package org.firstinspires.ftc.teamcode.cameraTest.HSVTest.Test1;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Point;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

//for dashboard
@Config
public class ConeDetectorPipeline extends OpenCvPipeline {

    // Backlog of frames to average out to reduce noise
    ArrayList<double[]> frameList;

    private static Telemetry telemetry;

    // Tuned in dashboard
    public static double strictLowH = 110;
    public static double strictHighH = 150;

    public static double strictLowS = 100;
    public static double strictHighS = 255;

    public static int dispStage = 0;

    int conePosXError = 0;

    public ConeDetectorPipeline() {
        frameList = new ArrayList<>();
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();

        // Convert from RGB to HSV
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }

        // Filters out near blue objects
        Scalar lowHSV = new Scalar(80, 90, 50); // lenient lower bound
        Scalar highHSV = new Scalar(150, 255, 255); // lenient higher bound

        Mat thresh = new Mat();

        // Get a black and white image of blue objects
        Core.inRange(mat, lowHSV, highHSV, thresh);

        Mat masked = new Mat();
        // Color the white portion of "thresh" in with HSV from "mat"
        // Output to "masked"
        Core.bitwise_and(mat, mat, masked, thresh);

        // Calculate average HSV values of the white "thresh" values
        Scalar average = Core.mean(masked, thresh);

        Mat scaledMask = new Mat();

        // Scale the average saturation to 150
        // This helps our strict bounding be more consistent
        masked.convertTo(scaledMask, -1, 200 / average.val[1], 0);

        Mat scaledThresh = new Mat();

        // Test this at comps and tune if necessary
        Scalar strictLowHSV = new Scalar(strictLowH, strictLowS, 50); //strict lower bound HSV for blue
        Scalar strictHighHSV = new Scalar(strictHighH, strictHighS, 255); //strict higher bound HSV for blue

        // Apply strict HSV filter to scaledMask to keep only cones/tape and ignore any other blue objects
        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

        Mat edges = new Mat();
        // Detect edges using Canny edge detection algorithm
        Imgproc.Canny(scaledThresh, edges, 100, 200);

        // Contours, apply post processing to information
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        // Using scaledThresh for the contour detection
        Imgproc.findContours(scaledThresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest contour to draw a box for.
        int area = 0, maxArea = 0;
        int cX = 0,cY = 0;
        int boxH = 0, boxW= 0;
        double ratio = 0;
        boolean foundLargest = false;
        int targetX = 0, targetY = 0 , targetW = 0, targetH = 0;

        // Iterate over each contour
        for (int i = 0; i < contours.size(); i++)
        {
            Rect rect = Imgproc.boundingRect(contours.get(i));  // Create a bounding rectangle

            // Get the measurements
            boxH = rect.height;
            boxW = rect.width;
            ratio = (double)boxW / (double)boxH;

            // If the box is too short, discard it as noise and move on to the next contour
            if (boxH < 10) {continue;}

            // Otherwise, continue with the algorithm

            // Generate the moments for the current contour
            Moments c = Imgproc.moments(contours.get(i));

            area = (int)Imgproc.contourArea(contours.get((i)));
            // If the contour we're currently looking at is larger than the current max
            // Replace the max area contour
            if (area > maxArea)
            {
                targetX = rect.x;
                targetY = rect.y;
                targetH = boxH;
                targetW = boxW;

                cX = (int)(c.m10 / c.m00);
                cY = (int)(c.m01 / c.m00);
                maxArea = area;
                // Flag to determine if we should draw a box or not
                foundLargest = true;
            }
        }

        // Only draw the box for the largest contour
        if (foundLargest) {
            conePosXError = cX - 640 / 2;

            // Add the text to the output
            Imgproc.putText(input,
                    String.valueOf(conePosXError) + " " + String.valueOf(maxArea) + " " + String.valueOf(0.6 <= ratio && ratio <= 1),
                    new Point(cX -5, cY - 5),
                    Imgproc.FONT_HERSHEY_SIMPLEX,0.5, new Scalar(255, 255, 255), 2);

            // Add the rectangle to the output
            Imgproc.rectangle(input,
                    new Point(targetX, targetY), // top left corner
                    new Point(targetX + targetW, targetY + targetH), // bottom right corner
                    new Scalar(0, 255, 0),
                    2);
        }


        // Backlog of 5 frames to average
        if (frameList.size() > 5) {
            frameList.remove(0);
        }

        // For changing the output mat in Dashboard (for viewing purposes only)
        if (1 <= dispStage && dispStage <= 6) {input.release();}

        switch (dispStage) {
            case 1:
                scaledThresh.copyTo(input);
                break;
            case 2:
                scaledMask.copyTo(input);
                break;
            case 3:
                mat.copyTo(input);
                break;
            case 4:
                masked.copyTo(input);
                break;
            case 5:
                edges.copyTo(input);
                break;
            case 6:
                thresh.copyTo(input);
                break;
            default:
                break;
        }

        //release all the unused data
        thresh.release(); // lenient filter
        mat.release(); // converted to HSV
        masked.release(); // filtered HSV
        scaledMask.release(); // normalized filtered HSV
        scaledThresh.release(); // strict filter
        edges.release(); // outlines

        return input; // display the selected Mat (which was copied to input)
    }
}