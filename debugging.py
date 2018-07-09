# -*- coding: utf-8 -*-
"""
Created on Tue Jun 26 13:43:04 2018

@author: GVKD1542
"""
from cap_constraint import find_components
from globals import *

def show_x(instance):
    expr = "   "
    for j in instance.nodes:
        expr += str(j) + "    "
    print(expr)
    for i in instance.nodes:
        expr = ""
        expr += str(i) + "  "
        for j in range(i):
            expr += ( str( (round(instance.x[i,j].value,2) )) if instance.x[i,j].value>eps else "-.-" ) + " "
        print(expr)
    
def explore_x(instance,i):
    sum = 0
    for j in instance.nodes:
        a,b = (max(i,j),min(i,j))
        if(a==b):
            continue
        print(str(j)+ " : " + str(round(instance.x[a,b].value,4)) + "  ")
        sum += instance.x[a,b].value
    print()
    print("total : " + str(sum))
	

def show_routes(instance):
	print()
	print("showing routes for found solution with capacity "+str(instance.capacity.value)+ " and "+str(instance.number_of_vehicles.value)+ " vehicles")
	print()
	comp = find_components(instance)
	for c in comp:
		expr = "0 "
		pred = 0
		total_load = 0
		total_cost = 0
		for node in c:
			total_cost += instance.costs[pred,node]
			total_load += instance.demands[node]
			expr+="-- "+str(round(instance.costs[pred,node],2))+" --> "
			expr+=str(node)+ " ("
			expr+=str(instance.demands[node])+") "
			pred = node
		expr+= "--> 0"
		print(expr)
		print("total load : "+str(total_load) + " and total cost : "+str(total_cost))
		print()
		