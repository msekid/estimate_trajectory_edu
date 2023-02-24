import os
import glob

class MakeTestList():
    def __init__(self):
        self.path = glob.glob('images/*.jpg')

    def print(self):
        print(self.path, "jpg")
    
    @staticmethod
    def makeData(self):
        data = []
        for f in self.path:
            data.append(os.path.split(f)[1])
            #print(os.path.split(f)[1])
        return sorted(data)

    def exportFile(self):
        with open('test_list.txt', 'w') as f:
            for i in MakeTestList.makeData(self):
                f.write("%s\n" % i)