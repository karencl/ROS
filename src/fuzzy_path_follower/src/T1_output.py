class output(object):

    def __init__(self, max_fs, interval):

        self._interval = interval
        self._max_fs = max_fs
        self._mean = (interval[1]+interval[0])/2.0

    @property
    def interval(self):
        return self._interval

    @property
    def mean(self):
        return self._mean

    @property
    def max_fs(self):
        return self._max_fs


class T1_Triangular_output(output):

    def get_degree(self, x):

        left = self._interval[0]
        right = self._interval[1]

        if (x <= left or x >= right):
            degree = 0.0
        elif (x == self._mean):
            degree = self._max_fs
        elif (x < self._mean):
            degree = min(((x - left) / (self._mean - left)), self._max_fs)
        elif (x > self._mean):
            degree = min(((right - x) / (right - self._mean)), self._max_fs)

        return (degree)


class T1_RightShoulder_output(output):

    def get_degree(self, x):
        if (x < self.interval[0] or x > self.interval[1]):
            return (min(0.0, self._max_fs))
        elif (x >= self.mean and x <= self.interval[1]):
            return (min(1.0, self._max_fs))
        elif (x < self.mean and x >= self.interval[0]):
            return (min(((x-self.interval[0])/float(self.mean - self.interval[0])), self._max_fs))
        else:
            raise ValueError("Something wrong with x in Right Shoulder.")


class T1_LeftShoulder_output(output):

    def get_degree(self, x):
        if (x > self.interval[1]):
            return (min(0.0, self._max_fs))
        elif (x <= self.mean):
            return (min(1.0, self._max_fs))
        elif (x > self.mean and x <= self.interval[1]):
            return (min(((self.interval[1]-x)/float(self.interval[1]-self.mean)), self._max_fs))
        else:
            raise ValueError("Something wrong with x in Left Shoulder.")
