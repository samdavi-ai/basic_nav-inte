"""YuNet 2023 decoder for the system OpenCV versions predating its output format."""
import cv2
import numpy as np
import onnxruntime as ort


class YuNet:
    def __init__(self, model):
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        self.session = ort.InferenceSession(str(model), options, providers=["CPUExecutionProvider"])
        self.input = self.session.get_inputs()[0]
        _, _, self.height, self.width = self.input.shape

    def detect(self, frame):
        h, w = frame.shape[:2]
        scale = min(self.width / w, self.height / h)
        size = (max(1, round(w * scale)), max(1, round(h * scale)))
        canvas = np.zeros((self.height, self.width, 3), np.uint8)
        canvas[:size[1], :size[0]] = cv2.resize(frame, size)
        heads = self.session.run(None, {self.input.name: cv2.dnn.blobFromImage(canvas)})
        outputs = {node.name: value[0] for node, value in zip(self.session.get_outputs(), heads)}
        faces = []
        for stride in (8, 16, 32):
            scores = np.sqrt(np.clip(outputs[f"cls_{stride}"] * outputs[f"obj_{stride}"], 0, 1)).ravel()
            for i in np.flatnonzero(scores >= .9):
                y, x = divmod(i, self.width // stride)
                anchor = np.array([x, y])
                box = outputs[f"bbox_{stride}"][i]
                center = (anchor + box[:2]) * stride
                if np.any(center < 0) or np.any(center >= size):
                    continue
                extent = np.exp(box[2:]) * stride
                landmarks = (outputs[f"kps_{stride}"][i].reshape(5, 2) + anchor) * stride
                coords = np.r_[center - extent / 2, extent, landmarks.ravel()]
                coords /= np.tile([size[0] / w, size[1] / h], 7)
                faces.append(np.r_[coords, scores[i]].astype(np.float32))
        if not faces:
            return []
        keep = cv2.dnn.NMSBoxes([f[:4].tolist() for f in faces], [float(f[-1]) for f in faces], .89, .3)
        return [faces[i] for i in np.asarray(keep).ravel()]
