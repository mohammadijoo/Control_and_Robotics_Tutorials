// Chapter13_Lesson4.java
// Robustness utilities: affine brightness fit and blur metric (variance of Laplacian).
// Dependencies: OpenCV Java bindings.
//
// Run (conceptual):
//   javac -cp .:opencv-xxx.jar Chapter13_Lesson4.java
//   java  -cp .:opencv-xxx.jar -Djava.library.path=<path_to_native_libs> Chapter13_Lesson4 img1.png img2.png

import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

public class Chapter13_Lesson4 {
    static { System.loadLibrary(Core.NATIVE_LIBRARY_NAME); }

    static double varLaplacian(Mat gray) {
        Mat lap = new Mat();
        Imgproc.Laplacian(gray, lap, CvType.CV_64F);
        MatOfDouble mean = new MatOfDouble();
        MatOfDouble std  = new MatOfDouble();
        Core.meanStdDev(lap, mean, std);
        double s = std.toArray()[0];
        return s * s;
    }

    static double[] affineBrightnessFit(Mat I1, Mat I2) {
        // Solve min_{a,b} || a*I1 + b - I2 ||^2 via normal equations (grayscale).
        if (I1.type() != CvType.CV_8U) I1.convertTo(I1, CvType.CV_8U);
        if (I2.type() != CvType.CV_8U) I2.convertTo(I2, CvType.CV_8U);

        double Sx = 0, Sy = 0, Sxx = 0, Sxy = 0;
        int rows = I1.rows(), cols = I1.cols();
        int N = rows * cols;

        byte[] buf1 = new byte[cols];
        byte[] buf2 = new byte[cols];

        for (int y = 0; y < rows; y++) {
            I1.get(y, 0, buf1);
            I2.get(y, 0, buf2);
            for (int x = 0; x < cols; x++) {
                double X = (buf1[x] & 0xFF);
                double Y = (buf2[x] & 0xFF);
                Sx += X; Sy += Y;
                Sxx += X * X;
                Sxy += X * Y;
            }
        }

        double det = Sxx * N - Sx * Sx;
        double a = 1.0, b = 0.0;
        if (Math.abs(det) >= 1e-12) {
            a = (Sxy * N - Sy * Sx) / det;
            b = (Sxx * Sy - Sx * Sxy) / det;
        }
        return new double[]{a, b};
    }

    public static void main(String[] args) {
        if (args.length < 2) {
            System.out.println("Usage: java Chapter13_Lesson4 img1 img2");
            return;
        }

        Mat img1 = Imgcodecs.imread(args[0], Imgcodecs.IMREAD_GRAYSCALE);
        Mat img2 = Imgcodecs.imread(args[1], Imgcodecs.IMREAD_GRAYSCALE);
        if (img1.empty() || img2.empty()) {
            System.out.println("Could not read images.");
            return;
        }
        Imgproc.resize(img2, img2, img1.size());

        double[] ab = affineBrightnessFit(img1, img2);
        double s1 = varLaplacian(img1);
        double s2 = varLaplacian(img2);

        System.out.println("Estimated affine brightness: a=" + ab[0] + " b=" + ab[1]);
        System.out.println("Blur score var(Laplacian): img1=" + s1 + " img2=" + s2);
    }
}
