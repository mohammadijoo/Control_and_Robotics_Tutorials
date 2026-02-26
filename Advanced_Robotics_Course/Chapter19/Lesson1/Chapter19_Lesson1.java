import ai.onnxruntime.*;
import java.nio.FloatBuffer;
import java.util.*;

public class VLAPolicy {
    private final OrtEnvironment env;
    private final OrtSession session;

    public VLAPolicy(String onnxPath) throws OrtException {
        env = OrtEnvironment.getEnvironment();
        OrtSession.SessionOptions opts = new OrtSession.SessionOptions();
        opts.setOptimizationLevel(OrtSession.SessionOptions.OptLevel.ALL_OPT);
        session = env.createSession(onnxPath, opts);
    }

    public float[] infer(float[] image, long[] imgShape,
                         float[] state, long[] stateShape,
                         long[] tokenIds, long[] tokenShape) throws OrtException {

        Map<String, OnnxTensor> inputs = new HashMap<>();
        inputs.put("image", OnnxTensor.createTensor(env, FloatBuffer.wrap(image), imgShape));
        inputs.put("state", OnnxTensor.createTensor(env, FloatBuffer.wrap(state), stateShape));
        inputs.put("tokens", OnnxTensor.createTensor(env, tokenIds, tokenShape));

        String[] outputNames = new String[]{"mu"};
        OrtSession.Result res = session.run(inputs, new HashSet<>(Arrays.asList(outputNames)));

        OnnxValue muVal = res.get("mu");
        float[] mu = (float[]) muVal.getValue();
        muVal.close();
        res.close();
        for (OnnxTensor t : inputs.values()) {
            t.close();
        }
        return mu; // continuous action vector
    }
}
      
