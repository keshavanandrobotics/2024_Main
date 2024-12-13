package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

import java.util.ArrayList;
import java.util.List;

@Autonomous
@Config
public class WebcamAutonomousExample extends LinearOpMode {
    OpenCvWebcam webcam;
    FtcDashboard dashboard;

    // Public static variables for HSV bounds
    public static double thetaM = 5;
    public static Scalar LOWER_RED1 = new Scalar(0, 120, 100);
    public static Scalar UPPER_RED1 = new Scalar(10, 255, 255);
    public static Scalar LOWER_RED2 = new Scalar(170, 130, 130);
    public static Scalar UPPER_RED2 = new Scalar(180, 255, 255);

    public static Scalar LOWER_YELLOW = new Scalar(10, 150, 120);
    public static Scalar UPPER_YELLOW = new Scalar(40, 255, 255);

    public static Scalar LOWER_BLUE = new Scalar(100, 150, 20);
    public static Scalar UPPER_BLUE = new Scalar(130, 255, 255);

    public static double PERCENTAGE_THRESHOLD = 10;

    public static double CONTOUR_SIZE = 5000.0; // Minimum contour area size to be considered
    public static boolean OUTPUT_IMAGE = true; // Toggle for output image

    @Override
    public void runOpMode() {
        // Initialize webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Initialize FTC Dashboard
        dashboard = FtcDashboard.getInstance();
        FtcDashboard.getInstance().startCameraStream(webcam, 0); // Stream the webcam to FTC Dashboard

        webcam.setPipeline(new SamplePipeline());

        // Open the camera asynchronously
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SENSOR_NATIVE);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", errorCode);
                telemetry.update();
            }
        });

        // Wait for start signal from Driver Station
        telemetry.addLine("Waiting for start");
        telemetry.update();
        waitForStart();

        // Autonomous logic starts here
        while (opModeIsActive()) {
            // Send telemetry data to Driver Station


            // Stop streaming after a certain condition is met
            if (webcam.getFrameCount() > 1000) {
                webcam.stopStreaming();
            }
        }

        // Stop camera streaming when autonomous period is over
        webcam.stopStreaming();
    }

    class SamplePipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Convert image to HSV
            Mat hsvImage = new Mat();
            Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);

            // Define thresholds for red, yellow, and blue detection
            Mat maskRed1 = new Mat();
            Mat maskRed2 = new Mat();
            Mat maskYellow = new Mat();
            Mat maskBlue = new Mat();

            // Detect red, yellow, and blue pixels
            Core.inRange(hsvImage, LOWER_RED1, UPPER_RED1, maskRed1);
            Core.inRange(hsvImage, LOWER_RED2, UPPER_RED2, maskRed2);
            Core.inRange(hsvImage, LOWER_YELLOW, UPPER_YELLOW, maskYellow);
            Core.inRange(hsvImage, LOWER_BLUE, UPPER_BLUE, maskBlue);

            // Combine the red masks
            Mat combinedMaskRed = new Mat();
            Core.bitwise_or(maskRed1, maskRed2, combinedMaskRed);

            // Calculate total pixel count in the image
            double totalPixels = input.rows() * input.cols();

            // Calculate pixel areas for each color
            double redArea = Core.countNonZero(combinedMaskRed);
            double yellowArea = Core.countNonZero(maskYellow);
            double blueArea = Core.countNonZero(maskBlue);

            // Calculate percentage of the image each color covers
            double redPercentage = (redArea / totalPixels) * 100;
            double yellowPercentage = (yellowArea / totalPixels) * 100;
            double bluePercentage = (blueArea / totalPixels) * 100;

            TelemetryPacket packet = new TelemetryPacket();


            // Output dominant color to telemetry if it covers more than 10%
            if (redPercentage > PERCENTAGE_THRESHOLD || yellowPercentage > PERCENTAGE_THRESHOLD || bluePercentage > PERCENTAGE_THRESHOLD) {
                if (redPercentage > yellowPercentage && redPercentage > bluePercentage) {
                    telemetry.addData("Dominant Color", "Red");
                    packet.put("Dominant Color", "Red");

                } else if (yellowPercentage > redPercentage && yellowPercentage > bluePercentage) {
                    telemetry.addData("Dominant Color", "Yellow");
                    packet.put("Dominant Color", "Yellow");

                } else {
                    telemetry.addData("Dominant Color", "Blue");
                    packet.put("Dominant Color", "Blue");

                }
            } else {
                telemetry.addData("Dominant Color", "None");
                packet.put("Dominant Color", "None");

            }

            // Update telemetry
            telemetry.update();
            dashboard.sendTelemetryPacket(packet);


            // Create a blank output image
            Mat output = new Mat(input.size(), input.type(), new Scalar(255, 255, 255)); // Start with a white image

            // Draw contours for each color, filtering by size (contours are only for visualization)
            drawContours(input, combinedMaskRed, new Scalar(255, 0, 0)); // Blue for red contours
            drawContours(input, maskYellow, new Scalar(255, 255, 0)); // Yellow for yellow contours
            drawContours(input, maskBlue, new Scalar(0, 0, 255)); // Red for blue contours

            // Apply color mapping based on detected color
            colorMap(hsvImage, output, LOWER_RED1, UPPER_RED1, new Scalar(255, 0, 0)); // Red
            colorMap(hsvImage, output, LOWER_YELLOW, UPPER_YELLOW, new Scalar(255, 255, 0)); // Yellow
            colorMap(hsvImage, output, LOWER_BLUE, UPPER_BLUE, new Scalar(0, 0, 255)); // Blue

            // Display the output image based on OUTPUT_IMAGE variable
            return OUTPUT_IMAGE ? output : input;
        }

        private void drawContours(Mat image, Mat mask, Scalar color) {
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);
                double area = rotatedRect.size.area();

                if (area > CONTOUR_SIZE) { // Filter based on CONTOUR_SIZE
                    Point[] vertices = new Point[4];
                    rotatedRect.points(vertices);

                    double width = rotatedRect.size.width;
                    double height = rotatedRect.size.height;

                    double distance = (1.5 * 405) / (Math.min(width, height));
                    distance *= 1.2804;

                    double theta = (rotatedRect.center.x - 160) / 320;
                    theta *= thetaM;

                    double x = distance * Math.cos(theta);
                    double y = distance * Math.sin(theta);

                    // Send the width and height to telemetry
                    telemetry.addData("Contour Width (px)", width);
                    telemetry.addData("Contour Height (px)", height);
                    telemetry.addData("Distance (in)", distance);
                    telemetry.addData("cols", image.cols());
                    telemetry.addData("wid", rotatedRect.center.x);
                    telemetry.addData("x (in)", x);
                    telemetry.addData("y (in)", y);

                    telemetry.update();

                    for (int i = 0; i < 4; i++) {
                        Imgproc.line(image, vertices[i], vertices[(i + 1) % 4], color, 2);
                    }
                }
            }
        }

        private void colorMap(Mat hsvImage, Mat output, Scalar lowerBound, Scalar upperBound, Scalar color) {
            Mat mask = new Mat();
            Core.inRange(hsvImage, lowerBound, upperBound, mask);
            Mat colored = new Mat(output.size(), output.type(), color);
            colored.copyTo(output, mask);
        }
    }
}
