import numpy as np
import time

class LowPassFilter ():
    
    def __init__(self):
        self.para = 0
        self.mem = 0
        self.init = True
        pass
    
    def set_para(self,para):
        self.para = para

    def filtrage(self,x):
        if self.init:
            self.mem = x
            self.init = False
        xf= x*self.para + self.mem * (1-self.para)
        self.mem = xf
        return xf

    def reset(self):
        self.mem = 0
        self.init = False

class MedianeFilter():
    def __init__(self):

        self.mem1=0
        self.mem2 = 0
        self.mem3 = 0
        self.mem4 = 0

        self.init = True
        pass

    def filtrage(self,x_start,x):
        if self.init:
            self.mem1 = x_start[0]
            self.mem2 = x_start[1]
            self.mem3 = x_start[2]
            self.mem4 = x_start[3]
            self.init=False
        a = np.array([self.mem1, self.mem2, self.mem3, self.mem4, x])
        xf = np.median(a)
        self.mem1 = self.mem2
        self.mem2 = self.mem3
        self.mem3 = self.mem4
        self.mem4 = x
        return xf

    def reset(self):
        self.mem1 = 0
        self.mem2 = 0
        self.mem3 = 0
        self.mem4 = 0
        self.init = False






