#!/usr/bin/env python3

import cv2
import numpy as np
import yaml

class MapProcessor:
    """Handles map loading, preprocessing, contour extraction, and coordinate transforms."""
    
    def __init__(self, map_path, yaml_path, scale_factor=6.0):
        self.map_path = map_path
        self.yaml_path = yaml_path
        self.scale_factor = scale_factor
        self._load_map_and_meta()
        self._process_map()

    def _load_map_and_meta(self):
        # Load map image
        self.map_img = cv2.imread(self.map_path, cv2.IMREAD_UNCHANGED)
        if self.map_img is None:
            raise FileNotFoundError(f"❌ Map not found: {self.map_path}")

        # Load YAML metadata
        with open(self.yaml_path, "r") as f:
            meta = yaml.safe_load(f)
        self.resolution = meta["resolution"]
        self.origin = meta["origin"][:2]
        self.height, self.width = self.map_img.shape
        print(f"✅ Loaded map, resolution={self.resolution}, origin={self.origin}")

    def _process_map(self):
        """Applies blur, threshold, morphology, contour extraction, and creates overlay."""
        blur = cv2.GaussianBlur(self.map_img, (3, 3), 0)
        _, binary = cv2.threshold(blur, 180, 255, cv2.THRESH_BINARY_INV)

        kernel = np.ones((3, 3), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        canvas = np.ones(
            (int(self.height * self.scale_factor), int(self.width * self.scale_factor)), 
            dtype=np.uint8
        ) * 255

        for cnt in contours:
            cnt_scaled = np.array(cnt * self.scale_factor, dtype=np.int32)
            cv2.drawContours(canvas, [cnt_scaled], -1, (0,), thickness=2)

        nonwhite = np.where(canvas < 250)
        y_min, y_max = np.min(nonwhite[0]), np.max(nonwhite[0])
        x_min, x_max = np.min(nonwhite[1]), np.max(nonwhite[1])

        self.cropped = canvas[y_min:y_max, x_min:x_max]
        self.bounds = (x_min, y_min)
        self.overlay = self._create_overlay(self.cropped)

    def _create_overlay(self, gray_img):
        overlay = cv2.cvtColor(gray_img, cv2.COLOR_GRAY2BGR)
        overlay[np.all(overlay == [0, 0, 0], axis=-1)] = [0, 255, 0]
        return overlay

    def image_to_world(self, click_x, click_y):
        """Convert click pixel (overlay coords) → world (x,y) in meters."""
        x_min, y_min = self.bounds
        x_scaled = click_x + x_min
        y_scaled = click_y + y_min
        x_orig = x_scaled / self.scale_factor
        y_orig = y_scaled / self.scale_factor

        world_x = self.origin[0] + (x_orig * self.resolution)
        world_y = self.origin[1] + ((self.height - y_orig) * self.resolution)
        return round(world_x, 3), round(world_y, 3)