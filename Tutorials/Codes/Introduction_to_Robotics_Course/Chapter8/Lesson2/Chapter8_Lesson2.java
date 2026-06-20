import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.*;

public class CameraCalib {
    static { System.loadLibrary(Core.NATIVE_LIBRARY_NAME); }

    public static void main(String[] args) {
        Size patternSize = new Size(9,6);
        double squareSize = 0.025;

        List<Mat> objpoints = new ArrayList<>();
        List<Mat> imgpoints = new ArrayList<>();

        // Prepare planar object points
        MatOfPoint3f objp = new MatOfPoint3f();
        List<Point3> pts3 = new ArrayList<>();
        for (int y=0; y<patternSize.height; y++)
            for (int x=0; x<patternSize.width; x++)
                pts3.add(new Point3(x*squareSize, y*squareSize, 0.0));
        objp.fromList(pts3);

        List<String> images = Arrays.asList("calib_images/1.png","calib_images/2.png"); // extend
        Size imageSize = null;

        for (String fname : images) {
            Mat img = Imgcodecs.imread(fname);
            imageSize = img.size();
            Mat gray = new Mat();
            Imgproc.cvtColor(img, gray, Imgproc.COLOR_BGR2GRAY);

            MatOfPoint2f corners = new MatOfPoint2f();
            boolean ok = Calib3d.findChessboardCorners(gray, patternSize, corners);

            if (ok) {
                objpoints.add(objp);
                imgpoints.add(corners);
            }
        }

        Mat K = Mat.eye(3,3, CvType.CV_64F);
        Mat dist = Mat.zeros(8,1, CvType.CV_64F);
        List<Mat> rvecs = new ArrayList<>();
        List<Mat> tvecs = new ArrayList<>();

        double rms = Calib3d.calibrateCamera(objpoints, imgpoints, imageSize, K, dist, rvecs, tvecs);

        System.out.println("RMS error: " + rms);
        System.out.println("K:\n" + K.dump());
        System.out.println("dist:\n" + dist.dump());
    }
}
