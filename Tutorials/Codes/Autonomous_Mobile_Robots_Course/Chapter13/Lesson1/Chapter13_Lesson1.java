// Chapter13_Lesson1.java
/*
Autonomous Mobile Robots (Control Engineering)
Chapter 13 - Visual and Visual–Inertial SLAM (AMR Focus)
Lesson 1  - Visual Odometry for Mobile Robots

Minimal monocular VO in Java using OpenCV Java bindings.
This is a teaching-oriented skeleton:
  - ORB detect/extract
  - Descriptor match (ratio test)
  - Essential matrix (RANSAC)
  - recoverPose
  - Pose chaining (trajectory up to scale)

Prereqs:
  - Install OpenCV with Java bindings and set java.library.path accordingly.
  - Ensure opencv-xxx.jar is on the classpath.

Run (example):
  javac -cp .:opencv-490.jar Chapter13_Lesson1.java
  java  -cp .:opencv-490.jar -Djava.library.path=/path/to/opencv/lib Chapter13_Lesson1 --images /path/to/frames
*/

import org.opencv.core.*;
import org.opencv.features2d.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.imgcodecs.Imgcodecs;

import java.io.*;
import java.nio.file.*;
import java.util.*;

public class Chapter13_Lesson1 {

    static class Intrinsics {
        double fx = 525.0, fy = 525.0, cx = 319.5, cy = 239.5;
        Mat K() {
            Mat K = Mat.eye(3,3, CvType.CV_64F);
            K.put(0,0, fx); K.put(1,1, fy);
            K.put(0,2, cx); K.put(1,2, cy);
            return K;
        }
    }

    static List<String> listImages(String folder) throws IOException {
        List<String> files = new ArrayList<>();
        String[] exts = new String[]{".png",".jpg",".jpeg",".bmp"};
        try (DirectoryStream<Path> stream = Files.newDirectoryStream(Paths.get(folder))) {
            for (Path p : stream) {
                String name = p.toString().toLowerCase();
                for (String e : exts) {
                    if (name.endsWith(e)) { files.add(p.toString()); break; }
                }
            }
        }
        Collections.sort(files);
        return files;
    }

    static List<DMatch> ratioTest(List<MatOfDMatch> knn, double ratio) {
        List<DMatch> good = new ArrayList<>();
        for (MatOfDMatch m : knn) {
            DMatch[] arr = m.toArray();
            if (arr.length < 2) continue;
            if (arr[0].distance < ratio * arr[1].distance) good.add(arr[0]);
        }
        return good;
    }

    static Matx44d makeT(Mat R, Mat t, double scale) {
        Matx44d T = Matx44d.eye();
        for (int r=0; r<3; r++) for (int c=0; c<3; c++) T.put(r,c, R.get(r,c)[0]);
        T.put(0,3, scale * t.get(0,0)[0]);
        T.put(1,3, scale * t.get(1,0)[0]);
        T.put(2,3, scale * t.get(2,0)[0]);
        return T;
    }

    static double[] cameraCenterWorld(Matx44d Tcw) {
        // Cw = -R^T t
        double[][] R = new double[3][3];
        double[] t = new double[]{Tcw.get(0,3), Tcw.get(1,3), Tcw.get(2,3)};
        for (int r=0; r<3; r++) for (int c=0; c<3; c++) R[r][c] = Tcw.get(r,c);
        double[] C = new double[3];
        for (int i=0; i<3; i++) {
            C[i] = -(R[0][i]*t[0] + R[1][i]*t[1] + R[2][i]*t[2]);
        }
        return C;
    }

    public static void main(String[] args) throws Exception {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        String folder = null;
        for (int i=0; i<args.length; i++) {
            if (args[i].equals("--images") && i+1 < args.length) folder = args[i+1];
        }
        if (folder == null) {
            System.out.println("Usage: java Chapter13_Lesson1 --images <folder>");
            return;
        }

        Intrinsics intr = new Intrinsics();
        Mat K = intr.K();

        List<String> files = listImages(folder);
        if (files.size() < 2) throw new RuntimeException("Need at least 2 frames.");

        ORB orb = ORB.create(2000);

        Mat img0 = Imgcodecs.imread(files.get(0), Imgcodecs.IMREAD_GRAYSCALE);
        MatOfKeyPoint kp0 = new MatOfKeyPoint();
        Mat des0 = new Mat();
        orb.detectAndCompute(img0, new Mat(), kp0, des0);

        DescriptorMatcher bf = DescriptorMatcher.create(DescriptorMatcher.BRUTEFORCE_HAMMING);

        Matx44d Tcw = Matx44d.eye();
        List<double[]> traj = new ArrayList<>();
        traj.add(cameraCenterWorld(Tcw));

        for (int i=1; i<files.size(); i++) {
            Mat img1 = Imgcodecs.imread(files.get(i), Imgcodecs.IMREAD_GRAYSCALE);
            MatOfKeyPoint kp1 = new MatOfKeyPoint();
            Mat des1 = new Mat();
            orb.detectAndCompute(img1, new Mat(), kp1, des1);

            if (des0.empty() || des1.empty()) {
                kp0 = kp1; des0 = des1;
                continue;
            }

            List<MatOfDMatch> knn = new ArrayList<>();
            bf.knnMatch(des0, des1, knn, 2);
            List<DMatch> good = ratioTest(knn, 0.75);
            if (good.size() < 20) {
                kp0 = kp1; des0 = des1;
                continue;
            }

            KeyPoint[] k0 = kp0.toArray();
            KeyPoint[] k1 = kp1.toArray();
            List<Point> p0 = new ArrayList<>();
            List<Point> p1 = new ArrayList<>();
            for (DMatch m : good) {
                Point a = k0[m.queryIdx].pt;
                Point b = k1[m.trainIdx].pt;
                p0.add(a); p1.add(b);
            }
            MatOfPoint2f P0 = new MatOfPoint2f(); P0.fromList(p0);
            MatOfPoint2f P1 = new MatOfPoint2f(); P1.fromList(p1);

            Mat inliers = new Mat();
            Mat E = Calib3d.findEssentialMat(P0, P1, K, Calib3d.RANSAC, 0.999, 1.0, inliers);
            if (E.empty()) {
                kp0 = kp1; des0 = des1;
                continue;
            }

            Mat R = new Mat();
            Mat t = new Mat();
            Calib3d.recoverPose(E, P0, P1, K, R, t, inliers);

            Matx44d T12 = makeT(R, t, 1.0);
            Tcw = T12.mul(Tcw); // NOTE: Matx.mul is elementwise; we want matrix multiply.

            // Workaround: explicit multiply for Matx44d
            double[][] A = new double[4][4];
            double[][] B = new double[4][4];
            for (int r=0; r<4; r++) for (int c=0; c<4; c++) { A[r][c] = T12.get(r,c); B[r][c] = Tcw.get(r,c); }
            double[][] C = new double[4][4];
            for (int r=0; r<4; r++) for (int c=0; c<4; c++) {
                double s = 0;
                for (int k=0; k<4; k++) s += A[r][k]*B[k][c];
                C[r][c] = s;
            }
            Matx44d Tnew = Matx44d.eye();
            for (int r=0; r<4; r++) for (int c=0; c<4; c++) Tnew.put(r,c, C[r][c]);
            Tcw = Tnew;

            traj.add(cameraCenterWorld(Tcw));
            kp0 = kp1; des0 = des1;

            if (i % 10 == 0 || i == files.size()-1) {
                double[] cw = traj.get(traj.size()-1);
                System.out.printf("[INFO] frame %d/%d Cw = %.3f, %.3f, %.3f%n", i, files.size()-1, cw[0], cw[1], cw[2]);
            }
        }

        try (PrintWriter out = new PrintWriter(new File(folder, "vo_trajectory_xyz.csv"))) {
            out.println("x,y,z");
            for (double[] p : traj) out.printf(Locale.US, "%f,%f,%f%n", p[0], p[1], p[2]);
        }
        System.out.println("[DONE] trajectory saved to: " + folder + "/vo_trajectory_xyz.csv");
    }
}
