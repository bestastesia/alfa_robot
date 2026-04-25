"""YOLO instance segmentation wrapper.

If model_path points to a valid .pt file, loads real YOLOv11-seg for inference.
Otherwise falls back to a stub that generates fake masks for integration testing.
"""

import numpy as np
from typing import List

from box_perception.utils import InstanceResult


class Segmentor:
    """YOLO-based instance segmentor for box detection."""

    def __init__(
        self,
        model_path: str = "",
        conf_threshold: float = 0.5,
        image_height: int = 480,
        image_width: int = 640,
    ):
        self.conf_threshold = conf_threshold
        self.h = image_height
        self.w = image_width
        self.model = None

        if model_path:
            try:
                from ultralytics import YOLO
                self.model = YOLO(model_path)
            except Exception as e:
                import warnings
                warnings.warn(f"Failed to load YOLO model from '{model_path}': {e}. "
                              "Falling back to stub.")

    def infer(self, rgb: np.ndarray) -> List[InstanceResult]:
        """Run instance segmentation on an RGB image.

        Args:
            rgb: (H, W, 3) uint8 BGR/RGB image.

        Returns:
            List of InstanceResult, one per detected box instance.
        """
        if self.model is not None:
            return self._infer_real(rgb)
        return self._infer_stub(rgb)

    def _infer_real(self, rgb: np.ndarray) -> List[InstanceResult]:
        """Real YOLOv11-seg inference path."""
        results_list = self.model(rgb, conf=self.conf_threshold, verbose=False)
        instances = []

        for results in results_list:
            if results.masks is None:
                continue
            masks = results.masks.data.cpu().numpy()       # (N, H_mask, W_mask)
            boxes = results.boxes
            h_img, w_img = rgb.shape[:2]

            for i in range(len(masks)):
                mask_raw = masks[i]
                # Resize mask to image size if needed
                if mask_raw.shape[0] != h_img or mask_raw.shape[1] != w_img:
                    import cv2
                    mask_raw = cv2.resize(mask_raw, (w_img, h_img),
                                          interpolation=cv2.INTER_LINEAR)
                mask_bool = mask_raw > 0.5

                xyxy = boxes.xyxy[i].cpu().numpy().astype(int)
                conf = float(boxes.conf[i].cpu().numpy())

                instances.append(InstanceResult(
                    box_id=i + 1,
                    mask=mask_bool,
                    bbox=(int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])),
                    confidence=conf,
                ))

        return instances

    def _infer_stub(self, rgb: np.ndarray) -> List[InstanceResult]:
        """Stub: generate 1-3 random fake masks covering the image center region.

        TODO: replace with real YOLO weights
        """
        h, w = rgb.shape[:2] if rgb.ndim >= 2 else (self.h, self.w)
        rng = np.random.RandomState()
        n_boxes = rng.randint(1, 4)  # 1 to 3 boxes
        results = []

        for i in range(n_boxes):
            # Random bbox in the central 60% of the image
            margin_x = int(w * 0.2)
            margin_y = int(h * 0.2)
            box_w = rng.randint(w // 6, w // 3)
            box_h = rng.randint(h // 6, h // 3)
            x1 = rng.randint(margin_x, max(margin_x + 1, w - margin_x - box_w))
            y1 = rng.randint(margin_y, max(margin_y + 1, h - margin_y - box_h))
            x2 = min(x1 + box_w, w)
            y2 = min(y1 + box_h, h)

            mask = np.zeros((h, w), dtype=bool)
            mask[y1:y2, x1:x2] = True

            confidence = float(rng.uniform(0.5, 0.99))

            results.append(InstanceResult(
                box_id=i + 1,
                mask=mask,
                bbox=(x1, y1, x2, y2),
                confidence=confidence,
            ))

        return results
