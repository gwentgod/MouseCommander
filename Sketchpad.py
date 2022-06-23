import cv2 as cv
import numpy as np

from MoveBaseClient import MoveBaseClient


class Sketchpad:
    def reset_path(self):
        self.path = np.empty((0, 2), dtype=np.uint)
        self.milestones = np.empty((0, 2))
        self.pad = np.copy(self.img)

    def __init__(self, img):
        self.img = img

        self.move_base_client = MoveBaseClient(self)
        self.move_base_client.reset()

        cv.namedWindow('Map')
        cv.setMouseCallback('Map', self.mouse_callback)

        self.mouse_down = False
        self.selected = None

        while cv.waitKey(1) & 0xff != 27:
            cv.imshow('Map', self.pad)

    def add_point(self, x, y):
        last_point = np.uint([[x, y]])
        self.path = np.concatenate((self.path, last_point))

        if self.path.shape[0] >= 2:
            cv.line(self.pad, tuple(self.path[-2]), tuple(self.path[-1]), color=(204, 102, 0), thickness=3)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv.EVENT_LBUTTONDOWN:
            self.mouse_down = True
            if self.milestones.size != 0:
                dist = np.linalg.norm(self.milestones - np.array([x, y]), axis=1)
                self.selected = dist.argmin()
                if dist[self.selected] > 20 or self.selected <= self.move_base_client.current_index:
                    self.selected = None

        elif event == cv.EVENT_MOUSEMOVE and self.mouse_down:
            if self.milestones.size == 0:
                self.add_point(x, y)
            elif self.selected is not None:
                self.milestones[self.selected] = np.array([x, y])
                self.mark_milestones()
 
        elif event == cv.EVENT_LBUTTONUP:
            if self.milestones.size == 0:
                self.get_milestones()
                self.mark_milestones()
            self.move_base_client.set_milestones(self.milestones)
            self.selected = None
            self.mouse_down = False

        elif event == cv.EVENT_RBUTTONDOWN:
            self.move_base_client.reset()

    def get_milestones(self, space=15):
        if self.path.size < space:
            self.milestones = np.array([self.path[0], self.path[-1]])
        else:
            self.milestones = self.path[::space]
            self.milestones[-1] = self.path[-1]

    def mark_milestones(self, current=0):
        self.pad = np.copy(self.img)
        for i in range(self.milestones.shape[0]):
            if i < self.move_base_client.current_index:
                color = (200, 100, 0)
            elif i == self.move_base_client.current_index:
                color = (0, 200, 100)
            else:
                color = (0, 128, 255)
            cv.circle(self.pad, tuple(self.milestones[i]), 5, color, -1)
            if i > 0:
                cv.line(self.pad, tuple(self.milestones[i - 1]), tuple(self.milestones[i]), color, 1)


if __name__ == '__main__':
    house = cv.imread("house.pgm")
    house = cv.resize(house, (house.shape[0]*2, house.shape[1]*2))
    Sketchpad(house)
