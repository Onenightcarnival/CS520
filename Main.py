import sys
from Controller import Controller
from classy_model import Model
from View import View

class Main(object):
    def __init__(self):
        self.TEST_FLAG = False
        self.strategy = 1
        self.run()

    def run(self):
        mobj = Model() 
        cobj = Controller()
        vobj = View()
        mobj.set_view(vobj)
        cobj.set_strategy(self.strategy)
        cobj.set_model(mobj)

if __name__ == '__main__':
    mobj = Main()
