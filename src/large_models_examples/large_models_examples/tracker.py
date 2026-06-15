import os
import cv2
import time
import torch
import functools
import numpy as np
import tensorrt as trt
import pycuda.autoinit 
import pycuda.driver as cuda
from collections import namedtuple

def timer(func):
    @functools.wraps(func)
    def wrapper(*args, **kwargs):
        # check start time
        start = time.perf_counter()

        # execute func
        value = func(*args, **kwargs)

        # check end time
        end = time.perf_counter()

        elapsed = end - start
        print('Finished {} in {} secs'.format(repr(func.__name__), round(elapsed, 7)))

        # bypass return
        return value

    return  wrapper


class StopWatch():
    def start(self):
        self.start_time = time.perf_counter()

    def stop(self):
        self.stop_time  = time.perf_counter()

    def __str__(self):
        return str(round(self.stop_time - self.start_time, 7))

class Config:
    TRACK_CONTEXT_AMOUNT     = 0.5
    TRACK_EXEMPLAR_SIZE      = 127
    TRACK_INSTANCE_SIZE      = 255
    TRACK_OUTPUT_SIZE        = 15

    TRACK_PENALTY_K          = 0.16
    TRACK_WINDOW_INFLUENCE   = 0.46
    TRACK_LR                 = 0.34

    POINT_STRIDE             = 8

config = Config
Corner = namedtuple('Corner', 'x1 y1 x2 y2')
Center = namedtuple('Center', 'x y w h')
def corner2center(corner):
    """ convert (x1, y1, x2, y2) to (cx, cy, w, h)
    Args:
        conrner: Corner or np.array (4*N)
    Return:
        Center or np.array (4 * N)
    """
    if isinstance(corner, Corner):
        x1, y1, x2, y2 = corner
        return Center((x1 + x2) * 0.5, (y1 + y2) * 0.5, (x2 - x1), (y2 - y1))
    else:
        x1, y1, x2, y2 = corner[0], corner[1], corner[2], corner[3]
        x = (x1 + x2) * 0.5
        y = (y1 + y2) * 0.5
        w = x2 - x1
        h = y2 - y1
        return x, y, w, h

class BackBoneProcessor(object):
    def __init__(self, input_dtype):
        if isinstance(input_dtype, list):
            self.in_dtype = input_dtype[0]
        else:
            self.in_dtype = input_dtype

        
    def pre(self, input):
        return np.ravel(input)

    def post(self, output):
        if len(output) != 1:
            raise ValueError("engine error!")

        return output[0]

class HeadProcessor(object):
    def __init__(self, input_dtype):
        if isinstance(input_dtype, list):
            self.in_dtype = input_dtype[0]
        else:
            self.in_dtype = input_dtype

    def pre(self, input1, input2):
        return [input1, input2]

    def post(self, output1, output2):
        # output1(zf) -> 1 * 2 * 16 * 16 -> cls
        # output2(xf) -> 1 * 4 * 16 * 16 -> loc
        output1 = torch.from_numpy(output1)
        output2 = torch.from_numpy(output2)

        output1 = torch.reshape(output1, [1,2,15,15])
        output2 = torch.reshape(output2, [1,4,15,15])

        return output1, output2

class Model(object):
    def __init__(self, exam_back_engine_path="", temp_back_engine_path="", head_engine_path=""):
        self.back_exam_engine = TRTEngine(exam_back_engine_path)
        self.back_temp_engine = TRTEngine(temp_back_engine_path)
        self.head_engine      = TRTEngine(head_engine_path)

        self.back_exam_processor = BackBoneProcessor(self.back_exam_engine.get_input_dtype())
        self.back_temp_processor = BackBoneProcessor(self.back_temp_engine.get_input_dtype())
        self.head_processor      = HeadProcessor(self.head_engine.get_input_dtype())
        

    def template(self, z):
        """
        args:
            z(ndarray): BGR image
        return:
            void
        """
        zf = self.back_temp_engine(self.back_temp_processor.pre(z))
        self.zf = self.back_temp_processor.post(zf)


    def track(self, x):
        """
        args:
            x(ndarray): BGR image
        return:
            {'cls': cls, 'loc': loc}
        """
        xf = self.back_exam_engine(self.back_exam_processor.pre(x))
        
        cls, loc = self.head_engine(self.head_processor.pre(self.zf, self.back_exam_processor.post(xf)))

        cls, loc = self.head_processor.post(cls, loc)

        return {
                'cls': cls,
                'loc': loc,
               }

class BaseTracker(object):
    def init(self, img, bbox):
        """
        img(np.ndarray): BGR image
        bbox: Bounding Box => [x, y, w, h]
        """
        raise NotImplementedError

    def track(self, img):
        """
        img(np.ndarray): BGR image
        bbox: Bounding Box => [x, y, w, h]
        """
        raise NotImplementedError

class SiameseTracker(BaseTracker):
    # BottleNeck Function... Time Consumer...
    # @timer
    def get_subwindow(self, im, pos, model_sz, original_sz, avg_chans):
        """
        im: BGR image
        pos: center position
        model_sz: exemplar size
        original_sz: original size
        avg_chans: channel average

        return: np.ndarray, not tensor
        """
        if isinstance(pos, float):
            pos = [pos, pos]

        sz = original_sz
        im_sz = im.shape
        c = (original_sz + 1) / 2
        # context_xmin = round(pos[0] - c) # py2 and py3 round
        context_xmin = np.floor(pos[0] - c + 0.5)
        context_xmax = context_xmin + sz - 1
        # context_ymin = round(pos[1] - c)
        context_ymin = np.floor(pos[1] - c + 0.5)
        context_ymax = context_ymin + sz - 1
        left_pad = int(max(0., -context_xmin))
        top_pad = int(max(0., -context_ymin))
        right_pad = int(max(0., context_xmax - im_sz[1] + 1))
        bottom_pad = int(max(0., context_ymax - im_sz[0] + 1))

        context_xmin = context_xmin + left_pad
        context_xmax = context_xmax + left_pad
        context_ymin = context_ymin + top_pad
        context_ymax = context_ymax + top_pad
        
        r, c, k = im.shape
        if any([top_pad, bottom_pad, left_pad, right_pad]):
            size = (r + top_pad + bottom_pad, c + left_pad + right_pad, k)
            te_im = np.zeros(size, np.uint8)
            te_im[top_pad:top_pad + r, left_pad:left_pad + c, :] = im
            if top_pad:
                te_im[0:top_pad, left_pad:left_pad + c, :] = avg_chans
            if bottom_pad:
                te_im[r + top_pad:, left_pad:left_pad + c, :] = avg_chans
            if left_pad: 
                te_im[:, 0:left_pad, :] = avg_chans
            if right_pad: 
                te_im[:, c + left_pad:, :] = avg_chans
            im_patch = te_im[int(context_ymin):int(context_ymax + 1),
                             int(context_xmin):int(context_xmax + 1), :]
        else:
            im_patch = im[int(context_ymin):int(context_ymax + 1),
                          int(context_xmin):int(context_xmax + 1), :]
        
        if not np.array_equal(model_sz, original_sz):
            im_patch = cv2.resize(im_patch, (model_sz, model_sz))
        im_patch = im_patch.transpose(2, 0, 1)
        im_patch = im_patch[np.newaxis, :, :, :]
        im_patch = im_patch.astype(np.float32)
        return im_patch

TRT_LOGGER = trt.Logger()

class TRTEngine(object):
    def __init__(self, engine_file_path=""):

        if (os.path.exists(engine_file_path)):
            print("Loading engine from path {}".format(engine_file_path))
            with open(engine_file_path, "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
                self.engine = runtime.deserialize_cuda_engine(f.read())
                self.context = self.engine.create_execution_context()
                self._allocate() 
        else:
            raise FileNotFoundError('Engine file {} is not found'.format(engine_file_path))


    def __call__(self, data):
        if isinstance(data, list):
            if len(data) != 2:
                raise ValueError("head error")
            
            self.inputs[0].host = data[0]
            self.inputs[1].host = data[1]

            [cuda.memcpy_htod_async(inp.device, inp.host, self.stream) for inp in self.inputs]

            for i in range(self.engine.num_io_tensors):
                self.context.set_tensor_address(self.engine.get_tensor_name(i), self.bindings[i])
            self.context.execute_async_v3(stream_handle=self.stream.handle)

            [cuda.memcpy_dtoh_async(out.host, out.device, self.stream) for out in self.outputs]

            self.stream.synchronize()

            return [out.host for out in self.outputs]
        else:
            self.inputs[0].host = data

            [cuda.memcpy_htod_async(inp.device, inp.host, self.stream) for inp in self.inputs]

            for i in range(self.engine.num_io_tensors):
                self.context.set_tensor_address(self.engine.get_tensor_name(i), self.bindings[i])
            self.context.execute_async_v3(stream_handle=self.stream.handle)

            [cuda.memcpy_dtoh_async(out.host, out.device, self.stream) for out in self.outputs]

            self.stream.synchronize()

            return [out.host for out in self.outputs]
        
    def _allocate(self):
        self.inputs = []
        self.outputs = []
        self.bindings = []
        self.stream = cuda.Stream()

        for i in range(self.engine.num_io_tensors):
            tensor_name = self.engine.get_tensor_name(i)
            size = trt.volume(self.engine.get_tensor_shape(tensor_name))
            dtype = trt.nptype(self.engine.get_tensor_dtype(tensor_name))
            # Allocate host and device buffers
            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)
            # Append the device buffer to device bindings.
            self.bindings.append(int(device_mem))
            # Append to the appropriate list.
            if self.engine.get_tensor_mode(tensor_name) == trt.TensorIOMode.INPUT:
                self.inputs.append(HostDeviceMem(host_mem, device_mem, dtype))
            else:
                self.outputs.append(HostDeviceMem(host_mem, device_mem, dtype))

    def get_input_dtype(self):
        return [ inp.dtype for inp in self.inputs ]

class HostDeviceMem(object):
    def __init__(self, host_mem, device_mem, dtype):
        self.host = host_mem
        self.device = device_mem
        self.dtype = dtype

    def __str__(self):
        return "Host:\n" + str(self.host) + "\nDevice:\n" + str(self.device) + "\nDtype:\n" + str(self.dtype)

    def __repr__(self):
        return self.__str__()

class Tracker(SiameseTracker):
    def __init__(self, back_exam_engine_path="", temp_exam_engine_path="", head_engine_path=""):
        self.score_size = config.TRACK_OUTPUT_SIZE

        hanning = np.hanning(self.score_size)
        window  = np.outer(hanning, hanning)
        self.cls_out_channels = 2
        self.window = window.flatten()

        self.points = self.generate_points(config.POINT_STRIDE, self.score_size)
        self.model = Model(back_exam_engine_path, temp_exam_engine_path, head_engine_path) 

    def generate_points(self, stride, size):
        ori = - (size // 2) * stride
        x, y = np.meshgrid([ori + stride * dx for dx in np.arange(0, size)],
                           [ori + stride * dy for dy in np.arange(0, size)])
        points = np.zeros((size * size, 2), dtype=np.float32)
        points[:, 0], points[:, 1] = x.astype(np.float32).flatten(), y.astype(np.float32).flatten()

        return points

    def _convert_bbox(self, delta, point):
        delta = delta.permute(1, 2, 3, 0).contiguous().view(4, -1)
        delta = delta.detach().cpu().numpy()

        delta[0, :] = point[:, 0] - delta[0, :] #x1
        delta[1, :] = point[:, 1] - delta[1, :] #y1
        delta[2, :] = point[:, 0] + delta[2, :] #x2
        delta[3, :] = point[:, 1] + delta[3, :] #y2
        delta[0, :], delta[1, :], delta[2, :], delta[3, :] = corner2center(delta)
        return delta

    def _convert_score(self, score):
        if self.cls_out_channels == 1:
            score = score.permute(1, 2, 3, 0).contiguous().view(-1)
            score = score.sigmoid().detach().cpu().numpy()
        else:
            score = score.permute(1, 2, 3, 0).contiguous().view(self.cls_out_channels, -1).permute(1, 0)
            score = score.softmax(1).detach()[:, 1].cpu().numpy()
        return score        

    def _bbox_clip(self, cx, cy, width, height, boundary):
        cx = max(0, min(cx, boundary[1]))
        cy = max(0, min(cy, boundary[0]))
        width = max(10, min(width, boundary[1]))
        height = max(10, min(height, boundary[0]))
        return cx, cy, width, height

    def init(self, img, bbox):
        """
        args:
            img(np.ndarray): BGR image
            bbox: Bounding Box => [x, y, w, h]
        return:
            void
        """
        # center_pos = [center x of bbox, center y of bbox]
        self.center_pos = np.array([bbox[0] + (bbox[2]-1) / 2,
                                    bbox[1] + (bbox[3]-1) / 2])

        # size = [w, h]
        self.size = np.array([bbox[2], bbox[3]])

        # z crop size 
        w_z = self.size[0] + config.TRACK_CONTEXT_AMOUNT * np.sum(self.size)
        h_z = self.size[1] + config.TRACK_CONTEXT_AMOUNT * np.sum(self.size)
        s_z = round(np.sqrt(w_z * h_z))

        # img channel average
        self.channel_average = np.mean(img, axis=(0, 1))

        # get z crop
        z_crop = self.get_subwindow(img, self.center_pos,
                                    config.TRACK_EXEMPLAR_SIZE,
                                    s_z, self.channel_average)

        # forward only backbone
        self.model.template(z_crop)

    def track(self, img):
        """
        args:
            img(np.ndarray): BGR image
        return:
            bbox(list): [x, y, width, height]
        """
        w_z = self.size[0] + config.TRACK_CONTEXT_AMOUNT * np.sum(self.size)
        h_z = self.size[1] + config.TRACK_CONTEXT_AMOUNT * np.sum(self.size)
        s_z = round(np.sqrt(w_z * h_z))
        scale_z = config.TRACK_EXEMPLAR_SIZE / s_z
        s_x = s_z * (config.TRACK_INSTANCE_SIZE / config.TRACK_EXEMPLAR_SIZE)

        # get x crop 
        x_crop = self.get_subwindow(img, self.center_pos,
                                    config.TRACK_INSTANCE_SIZE,
                                    round(s_x), self.channel_average)

        # forward all (backbone, head)
        outputs = self.model.track(x_crop)

        score = self._convert_score(outputs['cls'])
        bbox  = self._convert_bbox(outputs['loc'], self.points)


        def change(r):
            return np.maximum(r, 1. / r)

        def sz(w, h):
            pad = (w + h) * 0.5
            return np.sqrt((w + pad) * (h + pad))

        # scale penalty
        s_c = change(sz(bbox[2, :], bbox[3, :]) /
                    (sz(self.size[0]*scale_z, self.size[1]*scale_z)))

        # aspect ratio penalty
        r_c = change((self.size[0] / self.size[1]) /
                     (bbox[2, :] / bbox[3, :]))

        penalty = np.exp(-(r_c * s_c - 1) * config.TRACK_PENALTY_K)

        # score
        pscore = penalty * score

        # window penalty
        pscore = pscore * (1 - config.TRACK_WINDOW_INFLUENCE) + \
            self.window * config.TRACK_WINDOW_INFLUENCE

        best_idx = np.argmax(pscore)

        bbox = bbox[:, best_idx] / scale_z

        lr = penalty[best_idx] * score[best_idx] * config.TRACK_LR
        cx = bbox[0] + self.center_pos[0]
        cy = bbox[1] + self.center_pos[1]

        # smooth bbox
        width  = self.size[0] * (1 - lr) + bbox[2] * lr
        height = self.size[1] * (1 - lr) + bbox[3] * lr

        # clip boundary
        cx, cy, width, height = self._bbox_clip(cx, cy, width, height, img.shape[:2])

        # update state
        self.center_pos = np.array([cx, cy])
        self.size       = np.array([width, height])

        bbox = [cx - width / 2,
                cy - height / 2,
                width,
                height]

        best_score = score[best_idx]

        return best_score, list(map(int, bbox))

