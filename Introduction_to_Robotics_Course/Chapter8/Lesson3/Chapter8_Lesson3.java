import org.opencv.core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.features2d.*;

public class FeaturesDemo {
  static { System.loadLibrary(Core.NATIVE_LIBRARY_NAME); }

  public static void main(String[] args){
    Mat img = Imgcodecs.imread("scene.png", Imgcodecs.IMREAD_GRAYSCALE);

    // Edges
    Mat Ix = new Mat(), Iy = new Mat(), G = new Mat();
    Imgproc.Sobel(img, Ix, CvType.CV_64F, 1, 0);
    Imgproc.Sobel(img, Iy, CvType.CV_64F, 0, 1);
    Core.magnitude(Ix, Iy, G);
    Mat edges = new Mat();
    Imgproc.threshold(G, edges, 50, 255, Imgproc.THRESH_BINARY);

    // Corners (Harris)
    Mat harris = new Mat();
    Imgproc.cornerHarris(img, harris, 2, 3, 0.04);

    // Blobs
    SimpleBlobDetector detector = SimpleBlobDetector.create();
    MatOfKeyPoint keypoints = new MatOfKeyPoint();
    detector.detect(img, keypoints);

    System.out.println("blobs=" + keypoints.toArray().length);
  }
}
