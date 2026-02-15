public class FrontierSelector {

    public static final int UNKNOWN = -1;
    public static final int FREE = 0;
    public static final int OBSTACLE = 1;

    private static boolean isFrontier(int[][] grid, int i, int j) {
        if (grid[i][j] != FREE) return false;
        int[][] neigh = {
          {-1,0}, 
          {1,0}, 
          {0,-1}, 
          {0,1}
        };
        int rows = grid.length;
        int cols = grid[0].length;
        for (int[] d : neigh) {
            int ni = i + d[0];
            int nj = j + d[1];
            if (0 <= ni && ni < rows && 0 <= nj && nj < cols) {
                if (grid[ni][nj] == UNKNOWN) {
                    return true;
                }
            }
        }
        return false;
    }

    public static int[] selectClosestFrontier(int[][] grid, int ri, int rj) {
        double bestDist = Double.POSITIVE_INFINITY;
        int[] best = null;
        int rows = grid.length;
        int cols = grid[0].length;
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                if (isFrontier(grid, i, j)) {
                    double dx = i - ri;
                    double dy = j - rj;
                    double d = Math.sqrt(dx*dx + dy*dy);
                    if (d < bestDist) {
                        bestDist = d;
                        best = new int[]{i, j};
                    }
                }
            }
        }
        return best; // null means no frontier left
    }
}
      
