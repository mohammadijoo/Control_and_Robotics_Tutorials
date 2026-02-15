import org.opencv.core.*;
import org.opencv.videoio.VideoCapture;
import org.opencv.highgui.HighGui;

public class CameraBringUp {
    static { System.loadLibrary(Core.NATIVE_LIBRARY_NAME); }

    public static void main(String[] args) {
        VideoCapture cap = new VideoCapture(0);
        if (!cap.isOpened()) {
            System.out.println("Cannot open camera");
            return;
        }

        Mat frame = new Mat();
        while (true) {
            cap.read(frame);
            if (frame.empty()) break;

            HighGui.imshow("RGB stream", frame);
            if (HighGui.waitKey(1) == 'q') break;
        }
        cap.release();
        HighGui.destroyAllWindows();
    }
}
