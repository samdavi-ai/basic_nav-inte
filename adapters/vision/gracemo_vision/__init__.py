"""Lazy-load the optional webcam adapter so local face processing needs only OpenCV."""
__all__ = ["VisionAdapter", "main"]


def __getattr__(name):
    if name in __all__:
        from . import detector
        return getattr(detector,name)
    raise AttributeError(name)
