"""Local YuNet face detection and optional consent-enrolled SFace matching."""
import os
from pathlib import Path

import cv2
import numpy as np


class FaceRecognizer:
    def __init__(self, config):
        cfg = config.get("face_recognition", {})
        root = Path(__file__).resolve().parents[3]
        self.threshold = float(cfg.get("similarity_threshold", .55))
        self.margin = float(cfg.get("ambiguity_margin", .08))
        self.gallery_dir = cfg.get("gallery") or os.getenv("VIRA_FACE_GALLERY")
        self.detector = self.encoder = None
        self.ort_detector = False
        self.failed = False
        self.cascade = None
        self.embedding_path = root / cfg.get("embedding_path","models/face_recognition_sface_2021dec.onnx")
        model = root / cfg.get("model_path","models/face_detection_yunet_2023mar.onnx")
        if cfg.get("enabled", True) and model.is_file():
            if tuple(map(int, cv2.__version__.split(".")[:2])) < (4, 8) and model.name == "face_detection_yunet_2023mar.onnx":
                from .yunet import YuNet
                self.detector = YuNet(model)
                self.ort_detector = True
            else:
                self.detector = cv2.FaceDetectorYN.create(str(model),"",(320,320))

        cascade = root / "models/haarcascade_frontalface_default.xml"
        if cfg.get("enabled",True) and cascade.is_file():
            self.cascade = cv2.CascadeClassifier(str(cascade))

    def _faces(self, frame):
        if self.detector is None or frame is None or not frame.size:
            return []
        h,w = frame.shape[:2]
        if self.ort_detector:
            return self.detector.detect(frame)
        self.detector.setInputSize((w,h))
        try:
            _, faces = self.detector.detect(frame)
            self.failed = False
        except cv2.error:
            self.failed = True
            return []
        return [] if faces is None else faces

    def embedding(self, frame, face):
        x,y,w,h = map(int,face[:4])
        roi = frame[max(0,y):y+h,max(0,x):x+w]
        if min(w,h)<48 or not roi.size or cv2.Laplacian(roi,cv2.CV_64F).var()<40:
            return None
        if self.encoder is None:
            if not self.embedding_path.is_file():
                return None
            self.encoder = cv2.FaceRecognizerSF.create(str(self.embedding_path),"")
        aligned = self.encoder.alignCrop(frame,face)
        return self.encoder.feature(aligned).ravel().copy()

    def detect_faces(self, frame):
        results = []
        for face in self._faces(frame):
            x,y,w,h = map(int,face[:4])
            state, label = "unknown", None
            if self.gallery_dir:
                from adapters.perception.face_gallery import Gallery
                vector = self.embedding(frame,face)
                if vector is None:
                    state = "insufficient_quality_or_model"
                else:
                    state, label = Gallery(self.gallery_dir).match(vector,self.threshold,self.margin)
            results.append(dict(box=(x,y,w,h),confidence=float(face[-1]),identity=label or "unknown",
                                enrolled_face_identity_id=label, recognition_state=state, center_x=x+w/2))
        if (self.detector is None or self.failed) and self.cascade is not None and frame is not None and frame.size:
            gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            for x,y,w,h in self.cascade.detectMultiScale(gray,scaleFactor=1.1,minNeighbors=5,minSize=(28,28)):
                results.append(dict(box=tuple(map(int,(x,y,w,h))),confidence=None,identity="unknown",
                                    enrolled_face_identity_id=None,recognition_state="detection_only",
                                    center_x=float(x+w/2)))
        return results


if __name__ == "__main__":
    import argparse
    from adapters.perception.face_gallery import Gallery
    parser = argparse.ArgumentParser()
    parser.add_argument("action",choices=("enroll","delete"))
    parser.add_argument("label")
    parser.add_argument("--gallery",required=True)
    parser.add_argument("--image")
    parser.add_argument("--consent",action="store_true")
    args = parser.parse_args()
    gallery = Gallery(args.gallery)
    if args.action == "delete":
        if not gallery.delete(args.label):
            parser.exit(1,"Identity absent\n")
    else:
        if not args.consent or not args.image:
            parser.error("Consent and one local enrollment image are required")
        engine = FaceRecognizer({})
        frame = cv2.imread(args.image)
        faces = engine._faces(frame)
        if len(faces)!=1:
            parser.error("One detectable face is required")
        vector = engine.embedding(frame,faces[0])
        if vector is None:
            parser.error("Insufficient quality or compatible embedding model unavailable")
        gallery.enroll(args.label,vector,consent=True)
