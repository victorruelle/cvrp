from globals import *
from time import time
from time import strftime as stime
from support import list_to_string as ls

class timer():
	def __init__(self):
		self.origin = time()
		self.pred = self.origin
		self.curr = self.origin
	
	def lap(self):
		self.curr = time()
		elapsed = round(self.curr - self.pred,2)
		self.pred = time() #we don't take self.curr for more precision
		return str(elapsed)
		
	def empty_lap(self):
		self.cur = time()
	
	def print_elapsed(self):
		return(self.lap()+"s (global: "+str(round(time()-self.origin,2))+")")
		
	def reset(self):
		self = timer()
	
	def global_time(self):
		return time()-self.origin

time_record = timer()

def pprint(message):
	print(message)
	print(time_record.print_elapsed())
	print()
	
def reset_timer():
	time_record.reset()

def empty_time_lap():
	time_record.empty_lap()

def max_time_not_reached():
	return time_record.global_time() < max_global_time
	
class writer():
	def __init__(self,name=None):
		if name==None:
			self.name = stime("%d")+"-"+stime("%m")+"-"+stime("%Y")+"_"+stime("%H")+'h'+stime("%M")
		else:
			self.name = name
		self.full = self.name+".log"
		
	def write(self,message,id=None):
		file = open(self.full,"a")
		if id==None:
			file.write(message+"\n")
		else:
			file.write(ls(id)+": "+message+"\n")
		file.close()
	
	def write_timed(self,message,id=None):
		file = open(self.full,"a")
		if id==None:
			file.write("["+time_record.print_elapsed()+"] " + message+", "+"\n")
		else:
			file.write("["+time_record.print_elapsed()+"] " + ls(id)+": "+message+", "+"\n")
		file.close()
	
	def write_spaced(self,message,id=None):
		file = open(self.full,"a")
		if id==None:
			file.write("\n")
			file.write(message+"\n")
			# file.write("\n")
		else:
			file.write("\n")
			file.write(ls(id)+": "+message+"\n")
			# file.write("\n")
		file.close()
		
	def write_timed_spaced(self,message,id=None):
		file = open(self.full,"a")
		if id==None:
			file.write("\n")
			file.write("["+time_record.print_elapsed()+"] "+message+", "+time_record.print_elapsed()+"\n")
			# file.write("\n")
		else:
			file.write("\n")
			file.write("["+time_record.print_elapsed()+"] " + ls(id)+": "+message+", "+time_record.print_elapsed()+"\n")
			# file.write("\n")
		file.close()
	
	def title(self,name):
		empty_time_lap()
		file = open(self.full,"a")
		file.write("\n")
		file.write("\n")
		file.write("\n")
		file.write("++++++++++ "+name+"\n")
		file.write("\n")
		file.close()
	
	def subtitle(self,name,id=None):
		empty_time_lap()
		file = open(self.full,"a")
		if id==None:
			file.write("\n")
			file.write("+++++ "+name+"\n")
			# file.write("\n")
		else:
			file.write("\n")
			file.write("+++++ "+ls(id)+": "+name+"\n")
			# file.write("\n")
		file.close()
		
	def declare(self,value,name):
		self.write(name+": "+str(value))
	
	def write_globals(self):
		self.title("globals")
		self.declare(eps,"epsilon")
		self.declare(max_column_generation_count,"max number of column generation loops")
		self.declare(max_depth,"max depth of search tree")
		self.declare(max_time_for_shrinking,"maximum time spent for graph shrinking")
		self.declare(max_size_for_shrinking,"maximum size of a set that can be shrunk")
		self.declare(max_global_time,"maximum time spent for the cvrp solving")
		self.declare(vals,"threshold values iteratively used for the connected components heuristic")

class empty_writer():
#empty class used as a trick that will allow the main threads calling writer to not verify the state if debug boolean.
	def __init__(self,name=None):
		self.name = name
		
	def write(self,message,id=None):
		return 
	
	def write_timed(self,message,id=None):
		return
	
	def write_spaced(self,message,id=None):
		return
		
	def write_timed_spaced(self,message,id=None):
		return
	
	def title(self,name):
		return
	
	def subtitle(self,name,id=None):
		return
	
	def declare(self,value,name):
		return
		
	def write_globals(self):
		return

if debug :	
	log = writer()
else:
	log = empty_writer()