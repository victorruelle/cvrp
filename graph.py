# -*- coding: utf-8 -*-
"""
Created on Wed Jun 13 14:06:03 2018

@author: GVKD1542
"""

import numpy as np
import matplotlib.pyplot as plt
import random as rd
from scipy import optimize
import time
from globals import id, graphing
from logging_cvrp import log
from statistics import stats


class node:	
	
	def __init__(self,x,y,d,i,attr="client"):
		self.x = x
		self.y = y
		self.d = d
		self.index = i
		self.neighboors = []
		self.attribute = attr
		
	def add(self,j):
		if(type(j)==node and not(j in self.neighboors)):
			self.neighboors.append(j)
			j.neighboors.append(self)
			return True
		return False
	
	def deg(self):
		return len(self.neighboors)
	
	def show(self):
		if(self.attribute=="depot"):
			plt.plot([self.x],[self.y],'go',markersize=7)
		else:
			plt.plot([self.x],[self.y],'ro',markersize=7)
			plt.text(self.x-0.05,self.y-0.05,self.index)
		
		

class vertex:
	
	def __init__(self,i,j,c):
		#i and j must be type node
		if type(i)!=node or type(j)!=node:
			raise NameError("initialisation of vertex with something else than nodes!")
		self.i = i
		self.j = j
		self.cost = c
		self.passage = 0
	'''	
	def show(self):
		plt.plot([ self.i.x , self.j.x ] , [ self.i.y , self.j.y ],color='black',linestyle='--',linewidth=0.5)
		plt.text( (self.i.x +self.j.x)/2 , (self.i.y+self.j.y)/2 , self.cost )
	'''
	def show(self):
		if(self.passage>0):
			plt.plot([ self.i.x , self.j.x ] , [ self.i.y , self.j.y ], alpha=self.passage, color='black', linewidth=0.5)
			# plt.text( (0.7*self.i.x +0.3*self.j.x) , (0.7*self.i.y+0.3*self.j.y) , "cost : "+str(np.round(self.cost,2))+"\npassage : "+str(np.round(self.passage,2)) )
		
		
		
		
class Graph:
		
	def __init__(self,instance,locations):
		self.n = instance.n.value
		self.nodes = []
		self.vertices = {}
		for i in range(self.n):
			x,y = locations[i]
			self.nodes.append(node(x,y,instance.demands[i],i))
		for i in self.nodes:
			for j in self.nodes: 
				if i.index>j.index:
					self.vertices[(i.index,j.index)]= vertex(i,j,instance.costs[i.index,j.index])
		self.total_demand = sum(self.nodes[i].d for i in range(self.n))
		
	
	def show(self):
		for arr in self.vertices.values():
			if not(0 in [arr.i.index,arr.j.index]):
				arr.show()		  
		for p in self.nodes:
			p.show()
	
	def update_with_x(self,X):
		#X is meant to be an instance.x-0
		for bridge in X.keys():
			self.add_link(bridge[0],bridge[1],X[bridge].value)
	
	def add_link(self,i,j,value):
		#must update nodes and vertices to match new link made
		if i>j:
			self.nodes[i].add(j)
			self.vertices[i,j].passage += value
			
def full_graph(instance,locations,status,bypass=False,show_only=False):
	if graphing or bypass:
		log.write("saving "+status+" solution to "+log.name+"_"+status+"_solution_graph.png in current folder")
		g = Graph(instance,locations)
		g.update_with_x(instance.x)
		fig = plt.figure(figsize=(15,15))
		plt.axis('off')
		plt.title("graph of "+status+" solution found for CVRP solving with \n"+str(instance.n.value)+" nodes, "+str(instance.number_of_vehicles.value)+" vehicles with capacity of "+str(instance.capacity.value))
		g.show()
		# fig.show()
		if show_only:
			fig.show()
			return
		fig.savefig(log.name+"/"+status+"_solution_graph.png",bbox_inches='tight',dpi=fig.dpi*2)
		log.write_timed("finished saving graph")