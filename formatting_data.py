import sys
from logging_cvrp import log



def sanitize(string,seperator=""):
	chars = ['\t','\n',"_","-",":"]
	for c in chars:
		while c in string:
			i = string.index(c)
			try :
				string = string[:i]+seperator+string[i+len(c):]
			except IndexError:
				print("sanitize out of range with : ")
				print(string + "c = " + c + " i = "+ str(i) + "len = " + str(len(c)) )
	#we now remove consecutive seperators
	offset = 0
	for j in range(1,len(string)):
		if string[j-offset]==string[j-1-offset]==seperator:
			string = string[:j-1]+string[j:] #we remove index j-1
			offset += 1
	return string


def create_dat_smart(in_name,out_name):
	with open("C:\\Users\\GVKD1542\\Documents\\python\\instances\\"+in_name,"r") as f:
		"""reading .vrp data file"""
		''' supported formats : all formats from http://vrp.atd-lab.inf.puc-rio.br/index.php/en/ 
		letters must all be put to capital, all seperation characters (except . for commas) will be treated equally
		excepted arguments are : dimension, number of vehicles, capacity, locations or weights, demands '''
		lines = f.readlines()
		
		for i in range(len(lines)):
			line = sanitize(lines[i]," ")
			line = line.split(" ")
			while "" in line:
				line.remove("")
			lines[i] = line
		

		#expected_headers = ["NAME","COMMENT","TYPE","CVRP","DIMENSION","CAPACITY","DISTANCE","VEHICLES","EDGE WEIGHT TYPE","EDGE WEIGHT FORMAT","DISPLAY DATA TYPE","NODE COORD TYPE","NODE COORD SECTION","EDGE WEIGHT SECTION","DEMAND SECTION","DEPOT SECTION"]	

		
		#dimension
		found = False
		for line in lines:
			if line[0]=="DIMENSION":
				found = True
				try:
					dimension = int(float(line[-1]))
				except:
					raise NameError(line)
				break
		if not(found):
			raise "Could not locate DIMENSION parameter"
		
		#capacity
		found = False
		for line in lines:
			if line[0]=="CAPACITY":
				found = True
				try:
					capacity = int(float(line[-1]))
				except:
					raise NameError(line)
				break
		if not(found):
			raise "Could not locate DIMENSION parameter"

		#demands
		found = False
		start_index_demand = 0
		for i in range(len(lines)):
			line = lines[i]
			if line[0]=="DEMAND" and line[1]=="SECTION":
				start_index_demand = i+1
				found = True
				break
		if not(found):
			raise "Could not locate DEMAND SECTION parameter"
		
		#we want depot to be index 0 (needed for the coord/weights section)
		depot_set_apart = int(float(lines[start_index_demand][1]))>0
		indexing_offset = int(float(lines[start_index_demand][0])) - (1 if depot_set_apart else 0) 
		
		demands = [0]
		for i in range(0 if depot_set_apart else 1,dimension):
			line = lines[start_index_demand+i]
			try :
				demands.append(int(float(sanitize(line[1]))))
			except ValueError:
				print("error while adding demands with : ")
				print(start_index_demand+i)
				print(line)
				print(line[1])
				print(float(sanitize(line[1])))
			except IndexError:
				print("error while adding demands with : ")
				print(start_index_demand+i)
				print(line)
				print(line[1])
				print(float(sanitize(line[1])))


		#locations/weights
		found = False
		start_index = 0
		for i in range(len(lines)):
			line = lines[i]
			if line[0]=="NODE" and line[1]=="COORD" and line[2]=="SECTION":
				found = True
				start_index = i+1
				break 
		if found:
			input_type = "COORD"
			locations = {}
			line = lines[start_index]
			for i in range(dimension):
				line = lines[start_index+i]
				try:
					n,i,j = int(float(sanitize(line[0])))-indexing_offset,float(sanitize(line[1])),float(sanitize(line[2]))
				except IndexError:
					print("mauvaise indentation dans locations avec : ")
					print(line)
					print(n)
					print(i)
					print(j)
					print(len(line))
				locations[(n,0)] = i 
				locations[(n,1)] = j
			if depot_set_apart:
				depot_found = False
				depot_index = 0
				for i in range(len(lines)):
					line = lines[i]
					if line[0]=="DEPOT" and line[1]=="SECTION":
						depot_index = i+1
						break
				if not(depot_found):
					raise "could not locate DEPOT SECTION paremeter"
				line = lines[depot_index]
				locations[(0,0)] = float(line[0])
				locations[(0,1)] = float(line[1])
		else:
			for i in range(len(lines)):
				line = lines[i]
				if line[0]=="EDGE" and line[1]=="WEIGHT" and line[2]=="TYPE":
					start_index = i+1
					found = True
					break
			if not(found):
				raise "Could locate neither EDGE WEIGTH SECTION nor NODE COORD SECTION parameters"
			input_type = "WEIGHT"
			weights = {}
			for i in range(dimension):
				line = lines[start_index+i]
				for j in range(len(line)):
					try:
						weights[(i,j)] = float(line[j])
					except IndexError:
						raise "Error in writing of weights with paremeters "+str(i)+" "+str(j)
		

		#number of vehicles
		found = False
		for i in range(len(lines)):
			line = lines[i]
			if line[0]=="VEHICLES":
				found = True
				number_of_vehicles = int(float(line[-1]))
				break
		if not(found):
			for i in range(len(lines)):
				line = lines[i]
				if line[0]=="NAME" and line[-1][0]=="k":
					found = True
					number_of_vehicles = int(float(line[-1][1:]))
					break
		if not(found):
			number_of_vehicles = int( sum(demand for demand in demands)/capacity ) + 4
			log.write("could not locate VEHICLES. We calculated that we could use "+str(number_of_vehicles)+ "vehicles")

	''' constructing distances matrix if needed '''
	weights = {}
	if input_type == "COORD":
		for i in range(dimension):
			for j in range(dimension):
				weights[i,j] = pow( pow(locations[(i,0)]-locations[(j,0)],2) + pow(locations[(i,1)]-locations[(j,1)],2) , 0.5 )

	'''writing those variables in the right format'''

	with open("C:\\Users\\GVKD1542\\Documents\\python\\v09\\"+out_name,"w") as f:

		#number of vehicles
		f.write("param number_of_vehicles := " + str(number_of_vehicles) + ";\n")
		f.write("\n")
		
		#dimension
		f.write("param n := " + str(dimension) + ";\n")
		f.write("\n")
		
		#capacity
		f.write("param capacity := " + str(capacity) + ";\n")
		f.write("\n")
		
		#type of entry
		f.write("param entry_type := " + input_type + ";\n")
		f.write("\n")
		
		if input_type=="COORD":
			f.write("param locations := \n")
			for k in locations.keys():
				f.write(str(k[0])+ " "+str(k[1]) + " " + str(locations[k]) + "\n")
			f.write(";\n")
			f.write("\n")

			f.write("param costs := \n")
			for k in weights.keys():
					f.write(str(k[0])+ " "+str(k[1]) + " " + str(weights[k]) + "\n")
			f.write(";\n")
			f.write("\n")
		
		elif input_type=="WEIGHT":
			f.write("param locations := \n")
			for i in range(dimension):
				f.write(str(i)+" 0 -1" + "\n")
				f.write(str(i)+" 1 -1" + "\n")
			f.write(";\n")
			f.write("\n")

			f.write("param costs := \n")
			for k in weights.keys():
				f.write(str(k[0])+ " "+str(k[1]) + " " + str(weights[k]) + "\n")
			f.write(";\n")
			f.write("\n")
		
		else:
			raise "We screwed up : input_type not recognized at time of writing"
		
		#demands
		f.write("param demands := \n")
		for i in range(dimension):
			f.write(str(i) + " " + str(demands[i]) + "\n")
		f.write(";\n")
		
if __name__ == '__main__':
	if len(sys.argv) != 2 :
		raise "Entrez le nom du fichier à traduire et son nom de sortie"
	in_name = sys.argv[2]
	out_name = sys.argv[3]