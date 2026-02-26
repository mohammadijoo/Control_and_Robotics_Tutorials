import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

public class SimpleColorSegmentation {
    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    public static void main(String[] args) {
        String inputPath = "scene.png";
        Mat bgr = Imgcodecs.imread(inputPath);
        if (bgr.empty()) {
            System.err.println("Could not load image.");
            return;
        }

        Mat hsv = new Mat();
        Imgproc.cvtColor(bgr, hsv, Imgproc.COLOR_BGR2HSV);

        // Example: segment pixels that are "table" colored (e.g., brown)
        Scalar lower = new Scalar(5, 50, 50);
        Scalar upper = new Scalar(20, 255, 255);
        Mat mask = new Mat(hsv.rows(), hsv.cols(), CvType.CV_8UC1);
        Core.inRange(hsv, lower, upper, mask);

        Imgcodecs.imwrite("mask_table.png", mask);
        System.out.println("Saved binary mask to mask_table.png");
    }
}
      
