package Hardware;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class yellowpixelpipeolddd extends OpenCvPipeline {

    public static volatile Rect detectionRect;

    private Scalar lowerBound = new Scalar(22, 93, 0);
    private Scalar upperBound = new Scalar(45, 255, 255);

    private Scalar color = new Scalar(0, 255, 0);

    private Mat mask = new Mat();
    private Mat newMat = new Mat();
    private final List<MatOfPoint> contoursList = new ArrayList<>();

    private Rect JUNCTION = new Rect();

    MatOfPoint m = new MatOfPoint();
    double maxArea = 0, currentArea;
    int i;

    /**
     * Detects the biggest yellow cluster of pixels and puts a green rectangle around it.
     * TODO: something useful
     */

    @Override
    public Mat processFrame(Mat input) {

        Core.inRange(input, lowerBound, upperBound, mask); // Only select yellow pixels in range [lowerBound, upperBound]

        // Find yellow contours
        Imgproc.findContours(mask, contoursList, newMat, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the largest contour
        JUNCTION.set(null);
        if (contoursList.size() > 0) {
            maxArea = 0;
            for (i = 0; i < contoursList.size(); i++) {
                m = contoursList.get(i);
                currentArea = Imgproc.contourArea(m);
                if (currentArea > maxArea) {
                    maxArea = currentArea;
                    JUNCTION = Imgproc.boundingRect(m);
                }
            }
        }
        detectionRect = JUNCTION;

        if (JUNCTION != null) {
            Imgproc.rectangle(input, JUNCTION, color, 5);
        }

        for (MatOfPoint m : contoursList) {
            m.release();
        }
        contoursList.clear();

        mask.release();

        return input;
    }
}