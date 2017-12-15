import sys
from Controller import Controller
from classy_model import Model
from View import View
import argparse

class Main(object):
    def __init__(self, strategy):
        self.TEST_FLAG = False
        self.strategy = strategy
        self.run()

    def run(self):
        mobj = Model() 
        cobj = Controller()
        vobj = View()
        mobj.set_view(vobj)
        cobj.set_strategy(self.strategy)
        cobj.set_model(mobj)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Set a strategy [1,2,3] beware 1 is fast but only minimizes, default is 1 though')
    parser.add_argument('--strategy', help='an interger for stategy', default='1')
    args = parser.parse_args()
    stratint = args.strategy
    mobj = Main(int(stratint))
