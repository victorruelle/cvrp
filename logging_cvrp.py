import globals as g
from time import time
from time import strftime as stime
import os

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
		return(self.lap()+"s (global: "+str(round(self.global_time(),2))+")")
		
	def reset(self):
		self.origin = time()
		self.pred = self.origin
		self.curr = self.origin
	
	def global_time(self):
		return time()-self.origin

time_record = timer()

def pprint(message):
	print(time_record.print_elapsed())
	print()
	print(message)
	
def reset_timer():
	time_record.reset()

def empty_time_lap():
	time_record.empty_lap()

def max_time_not_reached():
	return time_record.global_time() < g.max_global_time
	
def global_time():
	return time_record.global_time()

class queue_logger():
	def __init__(self,log):
		self.name = log.name
		self.full = self.name + "/queue.log"
		self.full_tree = self.name + "/search_tree.log"

	def write(self,instance_manager):
		file = open(self.full,"a")
		message  = []
		for instance in instance_manager.queue:
			#message.append([instance.id,instance.objective_value])
			message.append([instance.objective_value])
		message = str(message)
		file.write(message+"\n")
		file.close()
	
class writer():
	def __init__(self,name=None):
		if name==None:
			self.name = stime("%d")+"-"+stime("%m")+"-"+stime("%Y")+"_"+stime("%H")+'h'+stime("%M")
		else:
			self.name = name
		self.name = "results/"+self.name
		self.full = self.name+"/record.log"
		if g.graphing or g.logging or g.search_tree in ["complete","end"] or g.graph_current_solution:
			os.makedirs(self.name)
		
	def write(self,message,id=None):
		file = open(self.full,"a")
		if id==None:
			file.write(message+"\n")
		else:
			file.write(list_to_string(	id)+": "+message+"\n")
		file.close()
	
	def write_timed(self,message,id=None):
		file = open(self.full,"a")
		if id==None:
			file.write(message+" "+"["+time_record.print_elapsed()+"] " + "\n")
		else:
			file.write(list_to_string(	id)+": "+message+" "+"["+time_record.print_elapsed()+"] " + "\n")
		file.close()
	
	def write_spaced(self,message,id=None):
		file = open(self.full,"a")
		if id==None:
			file.write("\n")
			file.write(message+"\n")
			# file.write("\n")
		else:
			file.write("\n")
			file.write(list_to_string(	id)+": "+message+"\n")
			# file.write("\n")
		file.close()
		
	def write_timed_spaced(self,message,id=None):
		file = open(self.full,"a")
		if id==None:
			file.write("\n")
			file.write(message+" "+time_record.print_elapsed()+"["+time_record.print_elapsed()+"] "+"\n")
			# file.write("\n")
		else:
			file.write("\n")
			file.write(list_to_string(	id)+": "+message+" "+time_record.print_elapsed()+"["+time_record.print_elapsed()+"] " +"\n")
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
			file.write("+++++ "+list_to_string(	id)+": "+name+"\n")
			# file.write("\n")
		file.close()

	def end_subtitle(self,name,id=None):
		empty_time_lap()
		file = open(self.full,"a")
		if id==None:
			file.write("\n")
			file.write("----- "+name+"\n")
			# file.write("\n")
		else:
			file.write("\n")
			file.write("----- "+list_to_string(	id)+": "+name+"\n")
			# file.write("\n")
		file.close()
		
	def declare(self,value,name):
		self.write(name+": "+str(value))
	
	def write_globals(self):
		self.title("globals")
		for var in dir(g):
			if not("__" in var) and var!="SolverFactory":
				self.declare(getattr(g,var),var)

	def write_awaiting_answer(self,message,id=None,timed=False):
		empty_time_lap()
		file = open(self.full,"a")
		if id==None:
			file.write(message+" "+(time_record.print_elapsed() if timed else "")+" ")
		else:
			file.write(list_to_string(	id)+": "+message+" "+(time_record.print_elapsed() if timed else "")+" ")
		file.close()
	
	def write_answer(self,message,id=None):
		self.write(message,id)
		
class empty_writer():
#empty class used as a trick that will allow the main threads calling writer to not verify the state if debug boolean.
	def __init__(self,name=None):
		if name==None:
			self.name = stime("%d")+"-"+stime("%m")+"-"+stime("%Y")+"_"+stime("%H")+'h'+stime("%M")
		else:
			self.name = name
		self.name = "results/"+self.name
		self.full = self.name+"/record.log"
		if g.graphing or g.logging or g.search_tree in ["complete","end"] or g.graph_current_solution :
			os.makedirs(self.name)
		
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

	def end_subtitle(self,name,id=None):
		return
	
	def declare(self,value,name):
		return
		
	def write_globals(self):
		return

	def write_awaiting_answer(self,message,id=None,timed=False):
		return
	
	def write_answer(self,message,id=None):
		return
		
class mixed_writer():
	def __init__(self,name=None):
		if name==None:
			self.name = stime("%d")+"-"+stime("%m")+"-"+stime("%Y")+"_"+stime("%H")+'h'+stime("%M")
		else:
			self.name = name
		self.name = "results/"+self.name
		self.full = self.name+"/record.log"
		if g.graphing or g.logging or g.search_tree in ["complete","end"] or g.graph_current_solution:
			os.makedirs(self.name)
		
	def write(self,message,id=None):
		file = open(self.full,"a")
		if id==None:
			file.write(message+"\n")
		else:
			file.write(list_to_string(	id)+": "+message+"\n")
		file.close()
	
	def write_timed(self,message,id=None):
		file = open(self.full,"a")
		if id==None:
			file.write(message+" "+"["+time_record.print_elapsed()+"] " + "\n")
		else:
			file.write(list_to_string(	id)+": "+message+" "+"["+time_record.print_elapsed()+"] " + "\n")
		file.close()
	
	def write_spaced(self,message,id=None):
		file = open(self.full,"a")
		if id==None:
			file.write("\n")
			file.write(message+"\n")
			# file.write("\n")
		else:
			file.write("\n")
			file.write(list_to_string(	id)+": "+message+"\n")
			# file.write("\n")
		file.close()
		
	def write_timed_spaced(self,message,id=None):
		file = open(self.full,"a")
		if id==None:
			file.write("\n")
			file.write(message+" "+time_record.print_elapsed()+"["+time_record.print_elapsed()+"] "+"\n")
			# file.write("\n")
		else:
			file.write("\n")
			file.write(list_to_string(	id)+": "+message+" "+time_record.print_elapsed()+"["+time_record.print_elapsed()+"] " +"\n")
			# file.write("\n")
		file.close()
	
	def title(self,name):
		pprint("++++++++++ "+name)
		empty_time_lap()
		file = open(self.full,"a")
		file.write("\n")
		file.write("\n")
		file.write("\n")
		file.write("++++++++++ "+name+"\n")
		file.write("\n")
		file.close()
	
	def subtitle(self,name,id=None):
		if id==None:
			pprint("+++++ "+name)
		else:
			pprint("+++++ "+list_to_string(	id)+": "+name)
		empty_time_lap()
		file = open(self.full,"a")
		if id==None:
			file.write("\n")
			file.write("+++++ "+name+"\n")
			# file.write("\n")file.close()
		else:
			file.write("\n")
			file.write("+++++ "+list_to_string(	id)+": "+name+"\n")
			# file.write("\n")
		file.close()
	
	def end_subtitle(self,name,id=None):
		if id==None:
			pprint("----- "+name)
		else:
			pprint("----- "+list_to_string(	id)+": "+name)
		empty_time_lap()
		file = open(self.full,"a")
		if id==None:
			file.write("----- "+name+"\n")
			# file.write("\n")file.close()
		else:
			file.write("----- "+list_to_string(	id)+": "+name+"\n")
			# file.write("\n")
		file.close()
	
	def declare(self,value,name):
		self.write(name+": "+str(value))
		
	def write_globals(self):
		self.title("globals")
		for var in dir(g):
			if not("__" in var) and var!="SolverFactory":
				self.declare(getattr(g,var),var)
	
	def write_awaiting_answer(self,message,id=None,timed=False):
		empty_time_lap()
		file = open(self.full,"a")
		if id==None:
			file.write(message+" "+(time_record.print_elapsed() if timed else "")+" ")
		else:
			file.write(list_to_string(	id)+": "+message+" "+(time_record.print_elapsed() if timed else "")+" ")
		file.close()
	
	def write_answer(self,message,id=None):
		self.write(message,id)

class print_writer:
	def __init__(self,name=None):
		if name==None:
			self.name = stime("%d")+"-"+stime("%m")+"-"+stime("%Y")+"_"+stime("%H")+'h'+stime("%M")
		else:
			self.name = name
		self.name = "results/"+self.name
		self.full = self.name+"/record.log"
		if g.graphing or g.logging or g.search_tree in ["complete","end"] or g.graph_current_solution:
			os.makedirs(self.name)
		
	def write(self,message,id=None):
		return
	
	def write_timed(self,message,id=None):
		return
	
	def write_spaced(self,message,id=None):
		return
		
	def write_timed_spaced(self,message,id=None):
		return
	
	def title(self,name):
		pprint("++++++++++ "+name)
	
	def subtitle(self,name,id=None):
		if id==None:
			pprint("+++++ "+name)
		else:
			pprint("+++++ "+list_to_string(	id)+": "+name)
		empty_time_lap()
	
	def end_subtitle(self,name,id=None):
		if id==None:
			pprint("----- "+name)
		else:
			pprint("----- "+list_to_string(	id)+": "+name)
		empty_time_lap()
	
	def declare(self,value,name):
		return
	
	def write_globals(self):
		return
	
	def write_awaiting_answer(self,message,id=None,timed=False):
		return
	
	def write_answer(self,message,id=None):
		return
	
def list_to_string(ls):
    	#returns more readible string of a list
	expr = ""
	for el in ls:
		expr += str(el)+"/"
	return expr[:-1]

if g.console_writing and g.logging :	
	log = mixed_writer()
elif not(g.console_writing) and g.logging:
	log = writer()
elif g.console_writing and not(g.logging):
	log = print_writer()
else:
	log = empty_writer()
queue_log = queue_logger(log)