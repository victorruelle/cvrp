# -*- coding: utf-8 -*-
"""
Created on Wed Jun 13 14:06:03 2018

@author: GVKD1542
"""

import numpy as np
from numpy import sqrt
import matplotlib.pyplot as plt
import random as rd
from scipy import optimize
import time
from globals import graphing, get_safe_counter
from logging_cvrp import log
import os

graph_counter = get_safe_counter()

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
	
	def __init__(self,i,j,c,fixed):
		#i and j must be type node
		if type(i)!=node or type(j)!=node:
			raise NameError("initialisation of vertex with something else than nodes!")
		self.i = i
		self.j = j
		self.cost = c
		self.passage = 0
		self.fixed = fixed
	'''	
	def show(self):
		plt.plot([ self.i.x , self.j.x ] , [ self.i.y , self.j.y ],color='black',linestyle='--',linewidth=0.5)
		plt.text( (self.i.x +self.j.x)/2 , (self.i.y+self.j.y)/2 , self.cost )
	'''
	def show(self,light=False):
		if(self.passage>0):
			if light:
				plt.plot([ self.i.x , self.j.x ] , [ self.i.y , self.j.y ], alpha=self.passage, color='red' if self.fixed else 'black', linewidth=0.5,linestyle=":")
			else:
				plt.plot([ self.i.x , self.j.x ] , [ self.i.y , self.j.y ], alpha=self.passage, color='red' if self.fixed else 'blue', linewidth=0.5)
				
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
					self.vertices[(i.index,j.index)]= vertex(i,j,instance.costs[i.index,j.index],instance.x[i.index,j.index].fixed)
		self.total_demand = sum(self.nodes[i].d for i in range(self.n))
		
	
	def show(self):
		for arr in self.vertices.values():
			if not(0 in [arr.i.index,arr.j.index]):
				arr.show()	
			else:
				arr.show(light=True)	  
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
	
class SearchTree:
	def __init__(self,instance_manager,old_nodes):
		node_pool = []
		for instance in instance_manager.queue:
			node_pool.append([instance,True])
		for instance in old_nodes:
			node_pool.append([instance,False])
		self.delta = (instance_manager.queue[-1].objective_value if len(instance_manager.queue)>0 else old_nodes[-1].objective_value) - old_nodes[0].objective_value
		self.nodes = {}
		self.n = instance_manager.length
		self.depth = 0 
		self.width = max( [ len( [ instance for instance,alive in node_pool if instance.depth == d ] ) for d in range(self.depth+1) ] )
		for instance,alive in node_pool:
			self.depth = max(self.depth,instance.depth)
		for instance,alive in node_pool:
			ancestry = instance.id
			for i in range(1,len(ancestry)+1):
				old_node_id = ancestry[:i]
				if not(hash_id(old_node_id) in self.nodes.keys()):
					if i==len(ancestry):
						self.nodes[hash_id(old_node_id)] = TreeNode(old_node_id,len(old_node_id),instance.objective_value,alive)
						if not(alive):
							self.nodes[hash_id(old_node_id)].order = old_nodes.index(instance)
					if i<len(ancestry):
						self.nodes[hash_id(old_node_id)] = TreeNode(old_node_id,len(old_node_id),None,False)
					if i>1:
						self.nodes[hash_id(old_node_id[:-1])].children.append(self.nodes[hash_id(old_node_id)])
				elif i==len(ancestry):
					self.nodes[hash_id(old_node_id)].objective_value = instance.objective_value
					if not(alive):
						self.nodes[hash_id(old_node_id)].order = old_nodes.index(instance)
					
		for node in self.nodes.values():
			node.construct_children()
		#print(self.nodes.keys())
		if len(self.nodes)>0:
			construct_width(self.nodes[hash_id([0])])
			for node in self.nodes.values():
				node.construct_coordinates(self.nodes)


	def show(self):
		if len(self.nodes)>0:
			fig = plt.figure(figsize=(max(4*self.nodes[hash_id([0])].width(),10),2+self.delta/30))
			plt.axis('off')
			plt.title("search tree")
			self.nodes[hash_id([0])].show()
			fig.savefig(log.name+"/search_tree.png",bbox_inches='tight',dpi=fig.dpi*2)
			'''
			try:
				fig.savefig(log.name+"/search_tree/"+str(graph_counter.get_and_increment())+".png",bbox_inches='tight',dpi=fig.dpi*2)
			except FileNotFoundError:
				os.makedirs(log.name+"/search_tree/")
				fig.savefig(log.name+"/search_tree/"+str(graph_counter.get_and_increment())+".png",bbox_inches='tight',dpi=fig.dpi*2)
			'''

def hash_id(id):
	output = ''
	for n in id:
		output+=str(n)
	return output

class TreeNode:
	space = 0.25

	def __init__(self,id,depth,objective_value,alive):
		self.id = id
		self.alive = alive
		self.depth = depth
		self.objective_value = objective_value
		self.children = []
		self.left = None
		self.right = None
		self.middle = None
		self.left_width = None
		self.right_width = None
		self.y = None
		self.x = None
		self.order = None
	
	def construct_children(self):
		if self.left!=None or self.middle!=None or self.right!= None or len(self.children)>3:
			raise EnvironmentError("This is not predicted")
		for child in self.children:
			last_id = child.id[-1]
			if (last_id[0]=='c' and last_id[1]=='1') or (last_id[0]=='i' and last_id[1]=='0'):
				self.left = child
			elif (last_id[0]=='c' and last_id[1]=='3') or (last_id[0]=='i' and last_id[1]=='1'):
				self.right = child
			elif (last_id[0]=='c' and last_id[1]=='2') or last_id=="0":
				self.middle = child
			else:
				raise Exception("constrution error in : construct_children (graph.py)")
	
	def construct_coordinates(self,nodes):
		if self.left_width == None or self.right_width == None:
			raise Exception("width was not initialised for instance "+str(self.id))
		self.y = - self.objective_value - self.depth*TreeNode.space/10 
		if self.id == [0]:
			self.x = 0
		else:
			parent = nodes[hash_id(self.id[:-1])]
			if parent.x == None:
				raise Exception("weird...")
			last_id = self.id[-1]
			if (last_id[0]=='c' and last_id[1]=='1') or (last_id[0]=='i' and last_id[1]=='0'):
				#self.x = parent.x - (self.right_width+ (0 if self.middle == None else self.middle.right_width ) )/2 - (0 if parent.middle==None else parent.middle.right_width)
				self.x = parent.x - self.right_width - ( 0 if parent.middle == None else parent.middle.left_width)
			elif (last_id[0]=='c' and last_id[1]=='3') or (last_id[0]=='i' and last_id[1]=='1'):
				#self.x = parent.x + (self.left_width+ (0 if self.middle == None else self.middle.left_width ) )/2 + (0 if parent.middle==None else parent.middle.right_width)
				self.x = parent.x + self.left_width + ( 0 if parent.middle == None else parent.middle.right_width)
			elif (last_id[0]=='c' and last_id[1]=='2') or last_id=="0":
				self.x = parent.x
			else:
				raise Exception("constrution error in : construct_children (graph.py)")

	def width(self):
		return self.left_width + self.right_width #+ self.middle_width

	def show(self):
		if self.alive:
			plt.plot([self.x],[self.y],'go',markersize=15/sqrt(self.depth))
		if not(self.alive):
			plt.plot([self.x],[self.y],'ro',markersize=15/sqrt(self.depth))
		if self.objective_value!=None:
			plt.text(self.x,self.y,str(round(self.objective_value,2))+"\n"+str(self.order if self.order!=None else "")+"\n",fontsize=15/sqrt(self.depth))
		for child in self.children:
			plt.plot([self.x,child.x],[self.y,child.y], color='black', linestyle=":",linewidth=2/sqrt(self.depth))
			child.show()

def construct_width(node):
	if node == None:
		return
		#return 0
	if node.children == []:
		last_id = node.id[-1]
		node.left_width = 0.25*TreeNode.space if last_id!=0 and ( (last_id[0]=='c' and last_id[1]=='1') or (last_id[0]=='i' and last_id[1]=='0') ) else 0.125*TreeNode.space
		node.right_width = 0.25*TreeNode.space if last_id!=0 and ( (last_id[0]=='c' and last_id[1]=='3') or (last_id[0]=='i' and last_id[1]=='1')  ) else 0.125*TreeNode.space
	else:
		construct_width(node.middle)
		construct_width(node.left)
		construct_width(node.right)
		node.left_width = (0.25*TreeNode.space if node.left == None else node.left.width()) + (0 if node.middle== None else node.middle.left_width ) 
		node.right_width = (0.25*TreeNode.space if node.right == None else node.right.width()) + (0 if node.middle== None else node.middle.right_width)
	#return node.left_width + node.right_width
			
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
		plt.close(fig)
		del(fig)