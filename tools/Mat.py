import scipy.io

class Mat(object):
    def load(self, filename, *args):
        tempdict=scipy.io.loadmat(filename)
        if len(args)==0:
            self.__dict__.update(tempdict)
        else:
            for arg in args:
                if isinstance(arg, str):
                    if arg in tempdict.keys():
                        self.__dict__[arg]=tempdict[arg]
                    else:
                        print('The specified variable was not included in the *.mat file.')
                else:
                    print('Variable names must be given as strings.')
                    
    def save(self,filename, *args):
        tempdict={}
        if len(args)==0:
            tempdict.update(self.__dict__)
        else:
            for arg in args:
                if isinstance(arg, str):
                    if arg in self.__dict__.keys():
                        tempdict[arg]=self.__dict__[arg]
                    else:
                        print('The specified variable could not be found in this instance.')
                else:
                    print('Variable names must be given as strings.')
        tempdict=scipy.io.savemat(filename, tempdict, oned_as='row')
    def clear(self):
        self.__dict__={}