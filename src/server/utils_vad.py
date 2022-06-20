import numpy as np
import onnxruntime


class OnnxWrapper():

    def __init__(self, path):
        self.session = onnxruntime.InferenceSession(path)
        self.session.intra_op_num_threads = 1
        self.session.inter_op_num_threads = 1

        self.reset_states()

    def reset_states(self):
        self._h = np.zeros((2, 1, 64)).astype('float32')
        self._c = np.zeros((2, 1, 64)).astype('float32')

    def __call__(self, x, sr: int):
        if x.ndim == 1:
            x = x[np.newaxis, ...]
        if x.ndim > 2:
            raise ValueError(f"Too many dimensions for input audio chunk {x.dim}")

        if sr != 16000 and (sr % 16000 == 0):
            step = sr // 16000
            x = x[::step]
            sr = 16000

        if x.shape[0] > 1:
            raise ValueError("Onnx model does not support batching")

        if sr not in [16000]:
            raise ValueError(f"Supported sample rates: {[16000]}")

        if sr / x.shape[1] > 31.25:
            raise ValueError("Input audio chunk is too short")

        ort_inputs = {'input': x, 'h0': self._h, 'c0': self._c}
        ort_outs = self.session.run(None, ort_inputs)
        out, self._h, self._c = ort_outs

        # out = torch.tensor(out).squeeze(2)[:, 1]  # make output type match JIT analog

        return out.reshape((1,-1))[:, 1][0]
