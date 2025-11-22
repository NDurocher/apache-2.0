import tensorrt as trt
import pycuda.autoinit
import pycuda.driver as cuda
import numpy as np


class TRTBadgrModel:
    def __init__(self, engine_path):
        TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

        with open(engine_path, "rb") as f:
            runtime = trt.Runtime(TRT_LOGGER)
            self.engine = runtime.deserialize_cuda_engine(f.read())

        print(f"Number of bindings: {self.engine.num_bindings}")
        for i in range(self.engine.num_bindings):
            name = self.engine.get_binding_name(i)
            shape = self.engine.get_binding_shape(i)
            dtype = self.engine.get_binding_dtype(i)
            print(f"Binding {i}: name={name}, shape={shape}, dtype={dtype}")
            self.context = self.engine.create_execution_context()

        # Get binding indices
        self.binding_names = [self.engine.get_binding_name(i) for i in range(self.engine.num_bindings)]

        # Replace your binding name lookups with:
        self.input_binding = "img"  # was "image"
        self.actions_binding = "onnx::MatMul_1"  # was "actions" 
        self.output_binding = "168"  # was "output"

        # Then use these to get indices and shapes:
        self.input_idx = self.engine[self.input_binding]
        self.actions_idx = self.engine[self.actions_binding]
        self.output_idx = self.engine[self.output_binding]

        self.shape_image = self.engine.get_binding_shape(self.input_idx)
        self.shape_actions = self.engine.get_binding_shape(self.actions_idx)
        self.shape_output = self.engine.get_binding_shape(self.output_idx)

        # And fix the Dims issue:
        shape_image_tuple = tuple(self.shape_image)
        shape_actions_tuple = tuple(self.shape_actions)
        shape_output_tuple = tuple(self.shape_output)

        self.d_image = cuda.mem_alloc(int(np.prod(shape_image_tuple) * 4))
        self.d_actions = cuda.mem_alloc(int(np.prod(shape_actions_tuple) * 4))
        self.d_output = cuda.mem_alloc(int(np.prod(shape_output_tuple) * 4))
        self.stream = cuda.Stream()

    def __call__(self, img_np, actions_np):
        # Flatten inputs
        img_flat = img_np.astype(np.float32).ravel()
        actions_flat = actions_np.astype(np.float32).ravel()

        # Copy to GPU
        cuda.memcpy_htod_async(self.d_image, img_flat, self.stream)
        cuda.memcpy_htod_async(self.d_actions, actions_flat, self.stream)

        # Run engine
        self.context.execute_async_v2(
            bindings=[int(self.d_image), int(self.d_actions), int(self.d_output)],
            stream_handle=self.stream.handle
        )

        # Copy output back
        out = np.empty(self.shape_output, dtype=np.float32)
        cuda.memcpy_dtoh_async(out, self.d_output, self.stream)
        self.stream.synchronize()

        return out

