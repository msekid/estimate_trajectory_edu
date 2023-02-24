import yaml

class Yaml():
    def __init__(self, camera_type):
        self.camera_type = camera_type

    def loadYaml(self):
        if (self.camera_type == 'denso'):
            with open('denso.yaml', 'r') as yml:
                config = yaml.safe_load(yml)
        elif (self.camera_type == 'horiba'):
            with open('horiba.yaml', 'r') as yml:
                config = yaml.safe_load(yml)
    
        return config

    def createYaml(self):
        if (self.camera_type == 'denso'):
            yml = [{'File.version':'1.0'},
                   {'Camera.type':'PinHole'},
                   {'Camera1.fx':532.67054732266013000},
                   {'Camera1.fy':532.15501999859521000},
                   {'Camera1.cx':644.19084542926225000},
                   {'Camera1.cy':363.13275822819526000},
                   {'Camera1.k1':-0.04346185407114724},
                   {'Camera1.k2':-0.03046945657935520},
                   {'Camera1.p1':0.00018878902723133},
                   {'Camera1.p2':0.00164609633216517},
                   {'Camera1.k3':0.00666543232313423},
                   {'Camera.fps':14},
                   {'Camera.RGB':1},
                   {'Camera.width':1280},
                   {'Camera.height':510},
                   {'ORBextractor.nFeatures':1200},
                   {'ORBextractor.scaleFactor':1.2},
                   {'ORBextractor.nLevels':10},
                   {'ORBextractor.iniThFAST':18},
                   {'ORBextractor.minThFAST':7},
                   {'Viewer.KeyFrameSize':0.05},
                   {'Viewer.KeyFrameLineWidth':1.0}]
            with open('denso_.yaml', 'w') as file:
                yaml.dump(yml, file)

Yaml(camera_type='denso').createYaml()