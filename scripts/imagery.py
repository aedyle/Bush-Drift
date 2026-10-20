"""Load the z18 imagery mosaic (from fetch_imagery.py) and sample it at lat/lon or local-frame points."""
import json
import os

import numpy as np
from PIL import Image
from scipy.ndimage import map_coordinates

from common import DATA, merc

Image.MAX_IMAGE_PIXELS = None


class Mosaic:
    def __init__(self):
        self.g = json.load(open(os.path.join(DATA, 'imagery_z18.json')))
        self.img = Image.open(os.path.join(DATA, 'imagery_z18.png')).convert('RGB')
        self.a = np.array(self.img)
        self.res = self.g['res']

    def pix(self, lat, lon):
        """Continuous pixel coords (col, row) of pixel *centres* convention: integer = centre."""
        mx, my = merc(lat, lon)
        return (mx - self.g['x0']) / self.res - 0.5, (self.g['y1'] - my) / self.res - 0.5

    def sample(self, lat, lon, order=1):
        c, r = self.pix(lat, lon)
        out = np.stack([map_coordinates(self.a[..., k], [r, c], order=order, mode='constant', cval=0)
                        for k in range(3)], -1)
        return out.astype(np.float32)
