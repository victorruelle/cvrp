''' accessing the value of a parameter : '''

#getting the parameters
from pyomo.core import Param
instance.component_objects(Param,active=True) # = iteratitor over the parameters (not constructed?)
str(p) = name of parameter # for p in instance. ... 

parameter = getattr(instance,"nameofparameter"/str(p))

for i in parameter :
	i = index of par (None if simple)
	parameter[i].value 
	

#looking into constraintlists
cs = getattre(instance,"c_deg") #for instance
len(cs)
cs.index_set()
keys = iterkeys(self) returns iterator over keys in the dict

#looking into a constraint
const.expr.
	to_string()
	polynomail_degree()
const.upperbound

#looking into results : dictionnary
print(results)
__getattr__(self,name)
__getitem__(self,name)
keys()
values()

#finding source code of a functions
from dill.source import getsource
#print(getsource(function))

#finding value of objective function
pyo.value(instance.objective)

#running a file from python interpreter
import sys
sys.argv = ['abc.py','arg1', 'arg2']
exec(open(file+".py").read(),globals())

#reloading a file/lib in python interpreter
import importlib
importlib.reload(file) #no quotation marks

#i can delete any component / instance / model with
del(model)
del(instance)
del(instance.set/par/var....)

#looking into dual set of values
instance.dual = pyo.Suffix(direction=pyo.Suffix.IMPORT)
instance.dual.items() # = generator of tuples(constraint, value of dual)
