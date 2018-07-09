import sys

''' construction names of files '''

if len(sys.argv) != 4 :
	raise "Enter 3 arguments : n, K, name of out file"

# in_name = "M-n"+sys.argv[1]+"-k"+sys.argv[2]+".vrp"
in_name = "E-n"+sys.argv[1]+"-k"+sys.argv[2]+".vrp"
# in_nam_solution = "X-n"+sys.argv[1]+"-k"+sys.argv[2]+".sol"
out_name = sys.argv[3]

'''constructing temporary variables'''


def sanitze(str):
	chars = ['\t','\n']
	for c in chars:
		while c in str:
			i = str.index(c)
			try :
				str = str[:i]+str[i+len(c):]
			except IndexError:
				print("sanitze out of range with : ")
				print(str + "c = " + c + " i = "+ str(i) + "len = " + str(len(c)) )
	return str

with open("C:\\Users\\GVKD1542\\Documents\\python\\Vrp-Set-X\\E\\"+in_name,"r") as f:
	"""reading .vrp data file"""
	lines = f.readlines()
	
	#values are seperated either by space or indent
	sep = ' ' if ' ' in lines[7] else '\t'
	
	
	#number of vehicles
	line = lines[0]
	line = line.split(":")
	line = line[1]
	line = line.split("-")
	word = line[2]
	number_of_vehicles = int(sanitze(word[1:]))
	
	#dimension
	line = lines[3]
	line = line.split(":")
	dimension = int(sanitze(line[1]))
	
	#capacity
	line = lines[5]
	line = line.split(":")
	capacity = int(sanitze(line[1]))
	
	locations_start_index = 7
	demands_start_index = locations_start_index + dimension + 1 #skip over title
	
	#locations
	locations = {}
	for i in range(dimension):
		line = lines[locations_start_index+i]
		line = line.split(sep)
		try:
			n,i,j = int(sanitze(line[0]))-1,int(sanitze(line[1])),int(sanitze(line[2]))#dans les fichiers vrp, l'indexation commence à 1
		except IndexError:
			print("mauvaise indentation dans locations avec : ")
			print(line)
			print(n)
			print(i)
			print(j)
			print(len(line))
		locations[(n,0)] = i 
		locations[(n,1)] = j 
	
	#demands
	demands = []
	for i in range(dimension):
		line = lines[demands_start_index+i]
		line = line.split(sep)
		try :
			demands.append(int(sanitze(line[1])))
		except ValueError:
			print("error while adding demands with : ")
			print(demands_start_index+i)
			print(line)
			print(line[1])
			print(sanitze(line[1]))

			
# with open("C:\\Users\\GVKD1542\\Documents\\python\\Vrp-Set-X\\X\\"+in_name_solution,"r") as f:
	# lines = f.readlines()
	# solution = int(lines[0])

# '''writing those variables in the right format'''

with open("C:\\Users\\GVKD1542\\Documents\\python\\v05\\"+out_name+".dat","w") as f:

	#number of vehicles
	f.write("param number_of_vehicles := " + str(number_of_vehicles) + ";\n")
	f.write("\n")
	
	#dimension
	f.write("param n := " + str(dimension) + ";\n")
	f.write("\n")
	
	#capacity
	f.write("param capacity := " + str(capacity) + ";\n")
	f.write("\n")
	
	#locations
	f.write("param locations := \n")
	for k in locations.keys():
		f.write(str(k[0])+ " "+str(k[1]) + " " + str(locations[k]) + "\n")
	f.write(";\n")
	f.write("\n")
	
	#demands
	f.write("param demands := \n")
	for i in range(dimension):
		f.write(str(i) + " " + str(demands[i]) + "\n")
	f.write(";\n")
	
	# solution
	# f.write("optimal solution := "+str(solution))
	
	
	
	
	