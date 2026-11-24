"""Private INT8 compatibility trial using alternating saved mapping frames for calibration."""
import sqlite3
import sys
from pathlib import Path

import cv2
import numpy as np
from onnxruntime.quantization import CalibrationDataReader, QuantType, quantize_static
from adapters.perception.detector import preprocess

db, model, output = map(Path, sys.argv[1:4])
if output.exists():
    raise FileExistsError(output)
with sqlite3.connect(db.resolve().as_uri()+"?mode=ro",uri=True) as conn:
    blobs = conn.execute("select image from Data where image is not null order by id").fetchall()[1::2]


class Calibration(CalibrationDataReader):
    def __init__(self):
        self.frames = iter(blobs)

    def get_next(self):
        row = next(self.frames,None)
        if row is None:
            return None
        frame = cv2.imdecode(np.frombuffer(row[0],np.uint8),cv2.IMREAD_COLOR)
        return {"images":preprocess(frame,640)[0]}


quantize_static(model,output,Calibration(),op_types_to_quantize=["Conv"],
                activation_type=QuantType.QInt8,weight_type=QuantType.QInt8,per_channel=True)
