import tensorrt as trt
import pycuda.autoinit
import pycuda.driver as cuda
print(f"CUDA context: {cuda.Context.get_current()}")
import numpy as np


class TRTBadgrModel:
    def __init__(self, engine_path, max_batch=50, max_horizon=10, verbose=True):
        self.verbose = verbose
        TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

        # Load engine
        with open(engine_path, "rb") as f:
            runtime = trt.Runtime(TRT_LOGGER)
            self.engine = runtime.deserialize_cuda_engine(f.read())

        self.context = self.engine.create_execution_context()

        # Map binding names to indices
        self.bindings = {
            self.engine.get_binding_name(i): i 
            for i in range(self.engine.num_bindings)
        }
        
        if self.verbose:
            print("[TRT] Engine bindings:")
            for name, idx in self.bindings.items():
                shape = self.engine.get_binding_shape(idx)
                dtype = self.engine.get_binding_dtype(idx)
                print(f"  {idx}: {name}, shape={shape}, dtype={dtype}")

        # Define names (adjust to match your actual engine bindings)
        # Based on your earlier error, these should be:
        self.input_name = "image"
        self.actions_name = "actions"
        self.output_name = "output"

        # Get binding indices
        self.input_idx = self.bindings[self.input_name]
        self.actions_idx = self.bindings[self.actions_name]
        self.output_idx = self.bindings[self.output_name]

        # Determine engine dtypes
        self.dtypes = {
            i: self.engine.get_binding_dtype(i) 
            for i in range(self.engine.num_bindings)
        }

        # Preallocate device buffers using max shapes
        self.max_shapes = {
            self.input_idx: (1, 3, 480, 640),        # Match your engine's expected input
            self.actions_idx: (max_batch, max_horizon, 2),
            self.output_idx: (max_batch, max_horizon, 1)
        }

        self.device_mem = {}
        for idx, shape in self.max_shapes.items():
            dtype = np.float16 if self.dtypes[idx] == trt.DataType.HALF else np.float32
            size_bytes = int(np.prod(shape) * np.dtype(dtype).itemsize)
            self.device_mem[idx] = cuda.mem_alloc(size_bytes)
            if self.verbose:
                print(f"[TRT] Allocated device buffer idx={idx}, shape={shape}, bytes={size_bytes}")

        # Single CUDA stream
        self.stream = cuda.Stream()

    def __call__(self, img_np, actions_np):
        """
        Run inference on the TensorRT engine.
        
        Args:
            img_np: Image tensor, shape (1, 3, 640, 480)
            actions_np: Actions tensor, shape (batch, horizon, 2)
        
        Returns:
            Output tensor as float32 numpy array
        """
        # Debug shapes
        print(f"[DEBUG] Input image shape: {img_np.shape}, expected: (1, 3, 640, 480)")
        print(f"[DEBUG] Input actions shape: {actions_np.shape}")
    
        # Validate shapes match engine expectations
        expected_img_shape = tuple(self.engine.get_binding_shape(self.input_idx))
        if img_np.shape != expected_img_shape:
            raise ValueError(f"Image shape mismatch: got {img_np.shape}, expected {expected_img_shape}") 

        # Ensure contiguous & correct dtype
        img_np = np.ascontiguousarray(img_np, dtype=np.float32)
        actions_np = np.ascontiguousarray(actions_np, dtype=np.float32)

        # Set dynamic binding shapes (if engine uses -1 dims)
        #self.context.set_binding_shape(self.input_idx, img_np.shape)
        #self.context.set_binding_shape(self.actions_idx, actions_np.shape)
        output_shape = self.max_shapes[self.output_idx]

        img_bytes = img_np.nbytes
        actions_bytes = actions_np.nbytes

        img_max_bytes = int(np.prod(self.max_shapes[self.input_idx]) * 4)
        actions_max_bytes = int(np.prod(self.max_shapes[self.actions_idx]) * 4)

        print(f"Image: {img_bytes} bytes, allocated: {img_max_bytes} bytes")
        print(f"Actions: {actions_bytes} bytes, allocated: {actions_max_bytes} bytes")

        if img_bytes > img_max_bytes or actions_bytes > actions_max_bytes:
            raise RuntimeError("Input exceeds allocated buffer size!")

        # Copy inputs to device
        cuda.memcpy_htod_async(
            self.device_mem[self.input_idx], 
            img_np.ravel(), 
            self.stream
        )
        cuda.memcpy_htod_async(
            self.device_mem[self.actions_idx], 
            actions_np.ravel(), 
            self.stream
        )

        # Execute
        bindings = [int(self.device_mem[idx]) for idx in range(self.engine.num_bindings)]
        self.context.execute_async_v2(
            bindings=bindings, 
            stream_handle=self.stream.handle
        )

        # Copy output back
        dtype = np.float16 if self.dtypes[self.output_idx] == trt.DataType.HALF else np.float32
        out = np.empty(output_shape, dtype=dtype)
        cuda.memcpy_dtoh_async(
            out, 
            self.device_mem[self.output_idx], 
            self.stream
        )
        self.stream.synchronize()

        return out.astype(np.float32)  # Convert to float32 for consistency
