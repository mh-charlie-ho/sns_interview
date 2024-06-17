import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time


class MonitorPlot():

    def __init__(self):
        self.fig, self.ax = plt.subplots()
        # plt.ion()
        self.v1 = []
        self.v2 = []
        self.t = []

        self.p1, = plt.plot([], [], 'ro')
        self.p2, = plt.plot([], [], 'ro')
        self.data1 = 0.0
        self.data2 = 0.0

    def initFigure(self):
        self.ax.set_xlim(0, 25)
        self.ax.set_ylim(0, 11000)

    def sendDate(self, v1, v2):
        self.data1 = v1
        self.data2 = v2

    def update(self, frame):
        # self.ax.set_xlim(-1, 50)
        self.v1.append(self.data1)
        self.v2.append(self.data2)
        self.t.append(time.time() - self.now)
        if (time.time() - self.now) >= 25 / 2:
            self.ax.set_xlim(time.time() - self.now - 12.5,
                             time.time() - self.now + 12.5)
        self.p1.set_data(self.t, self.v1)
        self.p2.set_data(self.t, self.v2)

    def animate(self):
        self.now = time.time()
        ani = FuncAnimation(self.fig,
                            self.update,
                            frames=None,
                            init_func=self.initFigure,
                            interval=100,
                            repeat=False)
        # plt.legend()
        plt.grid()
        plt.show()

