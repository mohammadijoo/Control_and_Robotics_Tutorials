// Chapter13_Lesson2.java
/*
Autonomous Mobile Robots — Chapter 13, Lesson 2
Feature-Based vs Direct Methods (Java / OpenCV)

Notes:
- Requires OpenCV Java bindings (org.opencv.*) in your classpath.
- This demo shows:
  (1) Feature-based: ORB + Essential matrix + recoverPose
  (2) Direct (sparse): KLT optical flow (photometric patch alignment) + robust transform estimate
*/
import org.opencv.core.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.features2d.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.video.Video;

import java.util.*;

public class Chapter13_Lesson2 {
    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    static Mat loadGray(String path) {
        Mat img = Imgcodecs.imread(path, Imgcodecs.IMREAD_GRAYSCALE);
        if (img.empty()) throw new RuntimeException("Could not read image: " + path);
        return img;
    }

    static class PoseResult {
        Mat R;
        Mat tUnit;
        Mat inliersMask;
        MatOfPoint2f pts1;
        MatOfPoint2f pts2;
    }

    static PoseResult featureBasedPose(Mat I1, Mat I2, Mat K) {
        ORB orb = ORB.create(2000);

        MatOfKeyPoint k1 = new MatOfKeyPoint();
        MatOfKeyPoint k2 = new MatOfKeyPoint();
        Mat d1 = new Mat(), d2 = new Mat();

        orb.detectAndCompute(I1, new Mat(), k1, d1);
        orb.detectAndCompute(I2, new Mat(), k2, d2);

        if (d1.empty() || d2.empty()) throw new RuntimeException("Not enough descriptors.");

        DescriptorMatcher matcher = DescriptorMatcher.create(DescriptorMatcher.BRUTEFORCE_HAMMING);
        List<MatOfDMatch> knn = new ArrayList<>();
        matcher.knnMatch(d1, d2, knn, 2);

        List<DMatch> good = new ArrayList<>();
        for (MatOfDMatch pair : knn) {
            DMatch[] ms = pair.toArray();
            if (ms.length < 2) continue;
            if (ms[0].distance < 0.75 * ms[1].distance) good.add(ms[0]);
        }
        if (good.size() < 8) throw new RuntimeException("Not enough good matches.");

        KeyPoint[] kp1 = k1.toArray();
        KeyPoint[] kp2 = k2.toArray();

        List<Point> p1 = new ArrayList<>();
        List<Point> p2 = new ArrayList<>();
        for (DMatch m : good) {
            p1.add(kp1[m.queryIdx].pt);
            p2.add(kp2[m.trainIdx].pt);
        }

        MatOfPoint2f pts1 = new MatOfPoint2f();
        MatOfPoint2f pts2 = new MatOfPoint2f();
        pts1.fromList(p1);
        pts2.fromList(p2);

        Mat inliers = new Mat();
        Mat E = Calib3d.findEssentialMat(pts1, pts2, K, Calib3d.RANSAC, 0.999, 1.0, inliers);
        if (E.empty()) throw new RuntimeException("findEssentialMat failed.");

        Mat R = new Mat();
        Mat t = new Mat();
        Calib3d.recoverPose(E, pts1, pts2, K, R, t, inliers);

        // Normalize translation (monocular scale unknown)
        double norm = Core.norm(t);
        Mat tUnit = (norm > 0) ? t.mul(new Scalar(1.0 / norm)) : t;

        PoseResult out = new PoseResult();
        out.R = R; out.tUnit = tUnit; out.inliersMask = inliers; out.pts1 = pts1; out.pts2 = pts2;
        return out;
    }

    static Point directSparseKLTTranslation(Mat I1, Mat I2) {
        // KLT solves a photometric alignment on small patches (direct / intensity-based)
        Mat corners = new Mat();
        Imgproc.goodFeaturesToTrack(I1, corners, 400, 0.01, 8.0);

        MatOfPoint2f p0 = new MatOfPoint2f(corners);
        MatOfPoint2f p1 = new MatOfPoint2f();
        MatOfByte status = new MatOfByte();
        MatOfFloat err = new MatOfFloat();

        Video.calcOpticalFlowPyrLK(I1, I2, p0, p1, status, err);

        Point[] a = p0.toArray();
        Point[] b = p1.toArray();
        byte[] st = status.toArray();

        // Robustly estimate translation by median flow
        List<Double> dx = new ArrayList<>();
        List<Double> dy = new ArrayList<>();
        for (int i = 0; i < st.length; i++) {
            if (st[i] == 0) continue;
            dx.add(b[i].x - a[i].x);
            dy.add(b[i].y - a[i].y);
        }
        if (dx.size() < 20) return new Point(0, 0);

        Collections.sort(dx);
        Collections.sort(dy);
        double medx = dx.get(dx.size()/2);
        double medy = dy.get(dy.size()/2);

        return new Point(medx, medy);
    }

    public static void main(String[] args) {
        if (args.length < 2) {
            System.out.println("Usage: java Chapter13_Lesson2 <img1> <img2> [fx fy cx cy]");
            return;
        }
        String img1 = args[0];
        String img2 = args[1];
        double fx = (args.length >= 3) ? Double.parseDouble(args[2]) : 525.0;
        double fy = (args.length >= 4) ? Double.parseDouble(args[3]) : 525.0;
        double cx = (args.length >= 5) ? Double.parseDouble(args[4]) : 319.5;
        double cy = (args.length >= 6) ? Double.parseDouble(args[5]) : 239.5;

        Mat I1 = loadGray(img1);
        Mat I2 = loadGray(img2);

        Mat K = Mat.eye(3, 3, CvType.CV_64F);
        K.put(0,0, fx); K.put(1,1, fy);
        K.put(0,2, cx); K.put(1,2, cy);

        System.out.println("=== Feature-based (ORB + Essential + recoverPose) ===");
        PoseResult pr = featureBasedPose(I1, I2, K);
        System.out.println("R:\n" + pr.R.dump());
        System.out.println("t (unit norm):\n" + pr.tUnit.dump() + " (monocular scale unknown)");

        System.out.println("\n=== Direct (sparse KLT photometric tracking) ===");
        Point uv = directSparseKLTTranslation(I1, I2);
        System.out.printf("Estimated pixel translation (median flow): u=%.3f, v=%.3f%n", uv.x, uv.y);
    }
}
