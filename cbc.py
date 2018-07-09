from __future__ import print_function
from ortools.linear_solver import pywraplp
import numpy as np
import support
import sys
import os



def lower_bound(instance):
	# Instantiate a mixed-integer solver, naming it SolveIntegerProblem.
	solver = pywraplp.Solver('SolveIntegerProblem',pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)

	
	n = instance.n.value
	
	#capacity
	capacity = instance.capacity.value
	
	#number of vehicles
	K = instance.number_of_vehicles

	# variable definitions.
	X = {(i,j):0 for i in range(n) for j in range(i) }

	for i in range(n):
		for j in range(i):
			if j==0:
				x = solver.IntVar(0.0,2.0,str(i)+":"+str(j))
			elif instance.x[i,j].fixed == True:
				x = solver.IntVar(instance.x[i,j].value,instance.x[i,j].value,str(i)+":"+str(j))
			else:
				x = solver.IntVar(0.0,1.0,str(i)+":"+str(j))
			X[(i,j)] = x

	# degree constraints
	C_deg = []
	for count in range(n):
		constraint = solver.Constraint(2 if count>0 else 2*K,2 if count>0 else 2*K)
		for k in X.keys():
			i,j = k
			if i==count or j==count:
				constraint.SetCoefficient(X[k],1)
			else:
				constraint.SetCoefficient(X[k],0)
		C_deg.append(constraint)
		

	# capacity constraints
	C_cap = []

	# Objective
	objective = solver.Objective()
	for k in X.keys():
		objective.SetCoefficient(X[k],instance.costs[k])
	objective.SetMinimization()

	"""Solve the problem and print the solution."""
	try:
		result_status = solver.Solve()
		
	except:
		raise NameError("pas de résultat")
	#assert result_status == pywraplp.Solver.OPTIMAL
	#assert solver.VerifySolution(1e-7, True)
	# The objective value of the solution.
	return solver.Objective().Value()

