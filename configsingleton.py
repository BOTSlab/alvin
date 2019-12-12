""" Not a true Singleton as this class can be created anywhere --- but if users
restrict themselves to calling 'get_instance' then it works as a Singleton in
essence.  Provides universal access to a single ConfigParser which reads from a config file upon the first call to 'get_instance'. """

import configparser, sys

class ConfigSingleton(configparser.ConfigParser, object):

    instance = None

    def __init__(self, config_file):
        super(ConfigSingleton, self).__init__()

        super(ConfigSingleton, self).read(config_file)

    @staticmethod
    def get_instance(config_file=None):

        if ConfigSingleton.instance == None and config_file == None:
            print("ConfigSingleton: First caller of 'get_instance' must \
                   supply a config file.")
            sys.exit(-1)
        elif ConfigSingleton.instance == None and config_file != None:
            ConfigSingleton.instance = ConfigSingleton(config_file)
        
        return ConfigSingleton.instance
