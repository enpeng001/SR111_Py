# point_cloud_viewer.py
import pyqtgraph.opengl as gl
import numpy as np

class PointCloudViewer(gl.GLViewWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('3D点云视图')
        self.setCameraPosition(distance=50)
        self.grid = gl.GLGridItem()
        self.addItem(self.grid)
        self.scatter = gl.GLScatterPlotItem(
            size=8,
            color=(1, 0, 0, 0.8),
            pxMode=False
        )
        self.addItem(self.scatter)
        self.filter_enabled = False
        self.filter_distance = 70  # 默认70米

    def update_points(self, points):
        """更新点云数据 points: [[x,y,z], ...]"""
        if self.filter_enabled and len(points) > 0:
            # 应用距离滤波
            distances = np.linalg.norm(points, axis=1)
            filtered = points[distances <= self.filter_distance]
            self.scatter.setData(pos=filtered)
        else:
            self.scatter.setData(pos=points)