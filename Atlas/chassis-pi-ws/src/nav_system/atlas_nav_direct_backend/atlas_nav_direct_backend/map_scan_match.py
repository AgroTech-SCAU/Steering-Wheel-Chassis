"""Check that a localized laser scan agrees with the saved occupancy map."""

from __future__ import annotations

import math
from pathlib import Path

import yaml

from .direct_nav_model import Pose2D


def _pgm_tokens(data: bytes):
    index = 0
    while index < len(data):
        while index < len(data) and data[index] in b" \t\r\n":
            index += 1
        if index < len(data) and data[index] == ord("#"):
            index = data.find(b"\n", index)
            if index < 0:
                return
            continue
        if index >= len(data):
            return
        end = index
        while end < len(data) and data[end] not in b" \t\r\n":
            end += 1
        yield data[index:end], end
        index = end


class MapScanMatch:
    def __init__(self, map_yaml: str) -> None:
        source = Path(map_yaml)
        config = yaml.safe_load(source.read_text(encoding="utf-8"))
        if (not isinstance(config, dict) or config.get("mode", "trinary") != "trinary"
                or int(config.get("negate", 0)) != 0):
            raise ValueError("only non-negated trinary occupancy maps are supported")
        self.resolution = float(config["resolution"])
        origin = config["origin"]
        self.origin = Pose2D(float(origin[0]), float(origin[1]), float(origin[2]))
        image = Path(config["image"])
        if not image.is_absolute():
            image = source.parent / image
        data = image.read_bytes()
        tokens = _pgm_tokens(data)
        magic, _ = next(tokens)
        width, _ = next(tokens)
        height, _ = next(tokens)
        maximum, end = next(tokens)
        if magic != b"P5" or int(maximum) != 255:
            raise ValueError("occupancy map must be an 8-bit P5 PGM")
        self.width = int(width)
        self.height = int(height)
        # P5 has one whitespace separator after the maximum value.
        self.pixels = data[end + 1:end + 1 + self.width * self.height]
        if len(self.pixels) != self.width * self.height or self.resolution <= 0:
            raise ValueError("occupancy map image is incomplete")
        self.occupied_gray = int(255 * (1.0 - float(config.get("occupied_thresh", 0.65))))
        self.free_gray = int(255 * (1.0 - float(config.get("free_thresh", 0.25))))
        # Cartographer's exported PGM uses 205 for unobserved cells. The YAML
        # free threshold may classify 205 as free, but no laser observation
        # should be compared against a cell that was never mapped.
        self.unknown_gray = 205

    def agreement(self, scan, map_to_laser: Pose2D) -> tuple[float, int]:
        """Return occupied endpoint fraction and number of comparable hits."""
        inverse_yaw = -self.origin.yaw
        cos_origin, sin_origin = math.cos(inverse_yaw), math.sin(inverse_yaw)
        radius = max(1, math.ceil(0.12 / self.resolution))
        comparable = matched = 0
        for index in range(0, len(scan.ranges), max(1, len(scan.ranges) // 180)):
            distance = float(scan.ranges[index])
            if not math.isfinite(distance) or distance < max(0.4, scan.range_min):
                continue
            if distance >= min(3.5, scan.range_max - 0.02):
                continue
            angle = map_to_laser.yaw + scan.angle_min + index * scan.angle_increment
            world_x = map_to_laser.x + distance * math.cos(angle) - self.origin.x
            world_y = map_to_laser.y + distance * math.sin(angle) - self.origin.y
            map_x = (cos_origin * world_x - sin_origin * world_y) / self.resolution
            map_y = (sin_origin * world_x + cos_origin * world_y) / self.resolution
            col = int(math.floor(map_x))
            row = self.height - 1 - int(math.floor(map_y))
            if not (radius <= col < self.width - radius and radius <= row < self.height - radius):
                continue
            nearby = [
                self.pixels[y * self.width + x]
                for y in range(row - radius, row + radius + 1)
                for x in range(col - radius, col + radius + 1)
            ]
            if not any(
                value <= self.occupied_gray or
                (value >= self.free_gray and value != self.unknown_gray)
                for value in nearby
            ):
                continue
            comparable += 1
            if any(value <= self.occupied_gray for value in nearby):
                matched += 1
        return (matched / comparable if comparable else 0.0, comparable)
