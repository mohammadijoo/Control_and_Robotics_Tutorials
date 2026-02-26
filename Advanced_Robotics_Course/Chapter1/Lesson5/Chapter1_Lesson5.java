public class OccupancyGrid2D {
    private final int width, height;
    private final double resolution;     // meters per cell
    private final boolean[][] occ;       // true = occupied

    // Obstacles as axis-aligned boxes
    public static class Box {
        public final double xmin, xmax, ymin, ymax;
        public Box(double xmin, double xmax, double ymin, double ymax) {
            this.xmin = xmin; this.xmax = xmax;
            this.ymin = ymin; this.ymax = ymax;
        }
    }

    private final java.util.List<Box> obstacles;
    private final double robotRadius;

    public OccupancyGrid2D(int width, int height, double resolution,
                           java.util.List<Box> obstacles,
                           double robotRadius) {
        this.width = width;
        this.height = height;
        this.resolution = resolution;
        this.obstacles = obstacles;
        this.robotRadius = robotRadius;
        this.occ = new boolean[width][height];
        precompute();
    }

    private void precompute() {
        for (int ix = 0; ix < width; ++ix) {
            for (int iy = 0; iy < height; ++iy) {
                double x = (ix + 0.5) * resolution;
                double y = (iy + 0.5) * resolution;
                occ[ix][iy] = isColliding(x, y);
            }
        }
    }

    private boolean isColliding(double x, double y) {
        for (Box b : obstacles) {
            double cx = Math.max(b.xmin, Math.min(x, b.xmax));
            double cy = Math.max(b.ymin, Math.min(y, b.ymax));
            double dx = x - cx;
            double dy = y - cy;
            if (dx * dx + dy * dy <= robotRadius * robotRadius) {
                return true;
            }
        }
        return false;
    }

    public boolean isFreeCell(int ix, int iy) {
        if (ix < 0 || ix >= width || iy < 0 || iy >= height) {
            return false; // treat outside as occupied
        }
        return !occ[ix][iy];
    }
}
      
