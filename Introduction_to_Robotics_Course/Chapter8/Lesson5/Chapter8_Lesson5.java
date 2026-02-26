import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.features2d.*;
import org.ejml.simple.SimpleMatrix;

public class PerceptionPipeline {
    static { System.loadLibrary(Core.NATIVE_LIBRARY_NAME); }

    public static void main(String[] args) {
        // (1) Acquire
        Mat img = Imgcodecs.imread("scene.png", Imgcodecs.IMREAD_GRAYSCALE);

        // (2) Preprocess
        Mat imgS = new Mat();
        Imgproc.GaussianBlur(img, imgS, new Size(5,5), 1.2);

        // (3) Feature extraction (goodFeaturesToTrack)
        Mat corners = new Mat();
        Imgproc.goodFeaturesToTrack(imgS, corners, 200, 0.01, 8);

        // (4) Weighted fusion with EJML
        SimpleMatrix x1 = new SimpleMatrix(2,1,true, new double[]{1.2,0.5});
        SimpleMatrix x2 = new SimpleMatrix(2,1,true, new double[]{1.0,0.9});
        SimpleMatrix S1 = SimpleMatrix.diag(0.04, 0.09);
        SimpleMatrix S2 = SimpleMatrix.diag(0.16, 0.04);

        SimpleMatrix W1 = S1.invert();
        SimpleMatrix W2 = S2.invert();
        SimpleMatrix xf = (W1.plus(W2)).invert().mult(W1.mult(x1).plus(W2.mult(x2)));

        System.out.println("Fused estimate: " + xf);
    }
}
