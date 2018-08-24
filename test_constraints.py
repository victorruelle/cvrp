import instance_managing as m
from globals import precision,pyo

'''
file to be used for the verification of the different heuristics from the C++ library
calling full_test(instance_to_test,instance_manager) will print all the constraints present
in instance_to_test that are not valid by testing them on a known solution. 
This could, for instance, be called after every column generation
'''


def test_instance(instance):
    clist = []
    for c,val in list(instance.dual.items()):
        lower = m.pyo.value(c.body) if m.pyo.value(c.lower) == None else m.pyo.value(c.lower)
        upper = m.pyo.value(c.body) if m.pyo.value(c.upper) == None else m.pyo.value(c.upper)
        if upper + precision >= pyo.value(c.body) >= lower - precision :
            continue
        clist.append(c)
    return clist

def show_bad_constraint(instance,cst):
    parent,index = cst.parent_component().name,cst.index()
    print("ctype: "+str(parent))
    print(str(pyo.value(cst.lower))+" <= "+str(pyo.value(cst.body))+" <= "+str(pyo.value(cst.upper)))

def show_bad_constraints(instance,clist):
    for cst in clist:
        show_bad_constraint(instance,cst)

def full_test(instance,valid_instance):
    instance0 = instance.clone()
    for k in instance0.x.keys():
        instance0.x[k].value = valid_instance.x[k].value
    clist = test_instance(instance0)
    show_bad_constraints(instance0,clist)