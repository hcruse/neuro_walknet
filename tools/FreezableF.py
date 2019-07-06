""" Took code from: http://code.activestate.com/recipes/252158-how-to-freeze-python-classes/ ."""
def freezable(set):
    """Raise an error when trying to set an undeclared name, or when calling
       from a method other than Frozen.__init__ or the __init__ method of
       a class derived from Frozen"""
    def set_attr(self,name,value):
        if hasattr(self,name) or not hasattr(self,'frozen') or self.frozen==False:                                  #If attribute already exists, simply set it
            set(self,name,value)
            return
        raise AttributeError("You cannot add attributes to %s after the object's attributes have been 'frozen'." % self)
    return set_attr

class Freezable(object):
    """Subclasses of Frozen are frozen, i.e. it is impossible to add
     new attributes to them and their instances."""
    __setattr__=freezable(object.__setattr__)
    class __metaclass__(type):
        __setattr__=freezable(type.__setattr__)
