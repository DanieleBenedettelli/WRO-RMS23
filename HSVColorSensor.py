""" Python package for HSV Color Sensor

Author  : Daniele Benedettelli
Date    : February 2023
Version : 0.1

"""

from pybricks.pupdevices import ColorSensor
from pybricks.parameters import Port, Color


class HSVColorSensor(ColorSensor):

    def __init__(self, port=Port.A):
        super().__init__(port)

    def getHSV(self):
        # r,g,b = self.rgb()
        (r, g, b) = self.read('RGB-RAW')
        _min = min(r, g, b)
        _max = max(r, g, b)
        # print("RGB RAW:%3d %3d %3d"%(r,g,b))

        value = int(_max)
        delta = _max - _min
        if delta == 0:
            saturation = 0
            hue = 0
        else:
            if _max > 0:
                saturation = int(100.0 * delta / _max)
            else:
                saturation = 0
                hue = 0

            if r == _max:
                hue = 0.0 + (g-b) / delta
            elif g == _max:
                hue = 2.0 + (b-r) / delta
            else:  # b == _max
                hue = 4.0 + (r-g) / delta
            hue *= 60.0
            while hue < 0:
                hue += 360
        hue = int(hue)
        return hue, saturation, value

    def getColor(self, longRange=True, printHSV = False):
        c = self.hsv()
        if printHSV:
            print("HSV   :%3d %3d %3d"%(c.h, c.s, c.v))
        reading = None
        """
        if longRange:
            if c.h > 65 and c.h < 170 and c.s > 60 and c.v > 2:
                reading = Color.GREEN
            elif c.h > 190 and c.h < 245 and c.s > 45 and c.v > 2 and c.v < 100:
                reading = Color.BLUE
            elif c.s > 30 and c.s < 55 and c.v > 20:
                reading = Color.WHITE
        else: #short range
        """
        if c.h > 65 and c.h < 170 and c.s > 45 and c.v > 4:
            reading = Color.GREEN
        elif c.h > 190 and c.h < 245 and c.s > 45 and c.v > 4: 
            reading = Color.BLUE
        elif c.s < 20 and c.v > 15: # v > 40
            reading = Color.WHITE
        #print("color:", reading)
        return reading

    def getRobustColor(self, colors=[Color.BLUE, Color.GREEN], samples=20, longRange=True):
        count = [0]*len(colors)
        for i in range(samples):
            sample = self.getColor(longRange)
            for k in range(len(colors)):
                if sample == colors[k]:
                    # increase count for the color that is being detected
                    count[k] += 1
        i = count.index(max(count))
        return colors[i]
