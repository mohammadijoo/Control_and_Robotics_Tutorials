
import java.util.*;

public class MobilityCounter {
    static Map<String,Integer> dof = Map.of(
        "R",1,"P",1,"H",1,"C",2,"U",2,"S",3,"E",3
    );

    static int mobilitySpatial(int nLinks, List<String> joints){
        int M = 6*(nLinks-1);
        for(String j : joints){
            M -= (6 - dof.get(j));
        }
        return M;
    }

    public static void main(String[] args){
        System.out.println(mobilitySpatial(3, Arrays.asList("R","P")));
    }
}
      