from __future__ import annotations

import numpy as np


class MujocoRgbdRenderer:
    """Renders aligned RGB and metric depth from one named MuJoCo camera."""

    def __init__(
        self,
        model,
        camera_name: str = "d435i_rgb",
        width: int = 640,
        height: int = 480,
    ):
        import mujoco

        self._mujoco = mujoco
        self.camera_name = camera_name
        self.width = int(width)
        self.height = int(height)
        self.renderer = mujoco.Renderer(
            model, height=self.height, width=self.width
        )

    def render_rgbd(self, data) -> tuple[np.ndarray, np.ndarray]:
        rgb = self.render_rgb(data)
        depth_m = self.render_depth(data, update_scene=False)
        return rgb, depth_m

    def render_rgb(self, data) -> np.ndarray:
        self.renderer.disable_depth_rendering()
        self.renderer.update_scene(data, camera=self.camera_name)
        return self.renderer.render().copy()

    def render_depth(self, data, *, update_scene: bool = True) -> np.ndarray:
        self.renderer.enable_depth_rendering()
        if update_scene:
            self.renderer.update_scene(data, camera=self.camera_name)
        depth_m = self.renderer.render().copy()
        self.renderer.disable_depth_rendering()
        return depth_m

    def close(self) -> None:
        self.renderer.close()

    def __enter__(self) -> "MujocoRgbdRenderer":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
