import cv2
import numpy as np
import os


class AroundViewTransformer:
    def __init__(self, world_x_range:tuple, world_x_interval:int, world_y_range:tuple, world_y_interval:int):
        self.roll = np.pi / 180 * 0
        self.pitch = np.pi / 180 * 16
        self.yaw = np.pi / 180 * 90
        self.x = 0.03
        self.y = 0
        self.z = 0.19
        self.intrinsic = np.array([[1.35791866e+03, 0, 8.80616797e+02, 0],
                                    [0, 1.32292093e+03, 6.52474295e+02, 0],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]])
        self.x_min, self.x_max = world_x_range
        self.x_interval = world_x_interval
        self.y_min, self.y_max = world_y_range
        self.y_interval = world_y_interval

        self.width = int(np.ceil((self.y_max - self.y_min) / self.y_interval))
        self.height = int(np.ceil((self.x_max - self.x_min) / self.x_interval))
        self.mat = np.array([[1.35791866e+03, 0.00000000e+00, 8.80616797e+02],
            [0.00000000e+00, 1.32292093e+03, 6.52474295e+02],
            [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]) 
        self.dist = np.array([[ 0.32550098, -0.95912881, 0.02602471, 0.01078656, 1.4634648 ]])
        h, w = 1232, 1640
        self.newmat, self.roi = cv2.getOptimalNewCameraMatrix(self.mat, self.dist, (w, h), 1, (w, h))
        self.intrinsic[:3, :3] = self.newmat

    def findRotation(self):
        """
        Get rotation matrix
        Args:
            roll, pitch, yaw:       In radians

        Returns:
            R:          [4, 4]
        """
        si, sj, sk = np.sin(self.roll), np.sin(self.pitch), np.sin(self.yaw)
        ci, cj, ck = np.cos(self.roll), np.cos(self.pitch), np.cos(self.yaw)
        cc, cs = ci * ck, ci * sk
        sc, ss = si * ck, si * sk

        R = np.identity(4)
        R[0, 0] = cj * ck
        R[0, 1] = sj * sc - cs
        R[0, 2] = sj * cc + ss
        R[1, 0] = cj * sk
        R[1, 1] = sj * ss + cc
        R[1, 2] = sj * cs - sc
        R[2, 0] = -sj
        R[2, 1] = cj * si
        R[2, 2] = cj * ci
        return R
    
    def findExtrinsic(self):
        translation = np.array([[1, 0, 0, -self.x],
                               [0, 1, 0, -self.y],
                               [0, 0, 1, -self.z],
                               [0, 0, 0, 1]])
        rotation = np.transpose(self.findRotation())
        R = np.array([[0, -1, 0, 0],
                      [0, 0, -1, 0],
                      [1, 0, 0, 0],
                      [0 ,0, 0, 1]])
        return R @ rotation @ translation

    def generateMap(self):
        # Create a grid of world coordinates
        world_x_coords = np.arange(self.x_max, self.x_min, -self.x_interval)
        world_y_coords = np.arange(self.y_max, self.y_min, -self.y_interval)
        world_x_grid, world_y_grid = np.meshgrid(world_x_coords, world_y_coords, indexing='ij')

        # Flatten the grids and add a column for homogeneous coordinates
        world_z = 0  # Assuming a flat plane at z=0
        ones = np.ones_like(world_x_grid.flatten())
        world_coords = np.vstack((world_x_grid.flatten(), world_y_grid.flatten(), 
                                np.full_like(ones, world_z), ones))

        # Transform to camera coordinates
        camera_coords = self.extrinsic[:3, :] @ world_coords

        # Transform to UV coordinates
        uv_coords = self.intrinsic[:3, :3] @ camera_coords
        uv_coords /= uv_coords[2]  # Normalize by the third coordinate (perspective division)


        # Reshape to match the map dimensions
        map_x = uv_coords[0].reshape(world_x_grid.shape).astype(np.float32)
        map_y = uv_coords[1].reshape(world_y_grid.shape).astype(np.float32)
        invalid_mask = (map_y < 400)
        map_x[invalid_mask] = -1
        map_y[invalid_mask] = -1
        return map_x, map_y
    
    def birdeyeRemap(self, img):
        self.extrinsic = self.findExtrinsic()
        self.map_x, self.map_y = self.generateMap()
        return cv2.remap(img, self.map_x, self.map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    
    def makeAroundView(self, imgs, dsize):
        canvas = np.zeros(shape=(dsize[1], dsize[0], 3), dtype=np.uint8)
        birdeye_imgs = []
        for i, img in enumerate(imgs):
            img = cv2.undistort(img, self.mat, self.dist, None, self.newmat)
            x, y, w, h = self.roi
            img = img[y:y+h, x:x+w]
            self.x = 0.03 * np.sin(np.pi / 2 - np.pi / 4 * i)
            self.y = 0.03 * np.cos(np.pi / 2 - np.pi / 4 * i)
            self.yaw = np.pi / 2 - np.pi / 4 * i
            img = self.birdeyeRemap(img)
            mask = cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY) < 10
            canvas[mask] = img[mask]
            birdeye_imgs.append(img)
        return canvas

