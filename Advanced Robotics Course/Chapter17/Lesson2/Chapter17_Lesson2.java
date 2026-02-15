import javax.vecmath.Vector3d;
import java.util.Arrays;

public class ClothState {
    private final int nx;
    private final int ny;
    private final Vector3d[][] positions;
    private final Vector3d[][] velocities;

    public ClothState(int nx, int ny) {
        this.nx = nx;
        this.ny = ny;
        this.positions = new Vector3d[nx][ny];
        this.velocities = new Vector3d[nx][ny];
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < ny; ++j) {
                positions[i][j] = new Vector3d(0.0, 0.0, 0.0);
                velocities[i][j] = new Vector3d(0.0, 0.0, 0.0);
            }
        }
    }

    public int stateDimension() {
        // 3 coordinates per node, 2 (position, velocity)
        return 6 * nx * ny;
    }

    public double[] toStateVector() {
        double[] s = new double[stateDimension()];
        int idx = 0;
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < ny; ++j) {
                Vector3d p = positions[i][j];
                s[idx++] = p.x;
                s[idx++] = p.y;
                s[idx++] = p.z;
            }
        }
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < ny; ++j) {
                Vector3d v = velocities[i][j];
                s[idx++] = v.x;
                s[idx++] = v.y;
                s[idx++] = v.z;
            }
        }
        return s;
    }

    public void fromStateVector(double[] s) {
        if (s.length != stateDimension()) {
            throw new IllegalArgumentException("Invalid state length");
        }
        int idx = 0;
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < ny; ++j) {
                positions[i][j].x = s[idx++];
                positions[i][j].y = s[idx++];
                positions[i][j].z = s[idx++];
            }
        }
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < ny; ++j) {
                velocities[i][j].x = s[idx++];
                velocities[i][j].y = s[idx++];
                velocities[i][j].z = s[idx++];
            }
        }
    }

    @Override
    public String toString() {
        return "ClothState(dim=" + stateDimension() + ")";
    }
}
      
