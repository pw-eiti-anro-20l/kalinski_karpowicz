# funkcja, read_DH(path_to_DH_table), tworzy plik params.yaml w folderze config i zwraca do niego sciezke.
from os.path import join
import itertools
import csv
import math
import PyKDL as pykdl

table = {'i':[], 'a':[], 'd':[], 'alfa':[], 'theta':[]}

def dh_2_rpy(table_name, config_file):
    lines_read = 0 #nie bierzemy pod uwage 1 wiersza

    with open(join('config', table_name), 'r') as f:
        reader = csv.DictReader(f, dialect=csv.unix_dialect)
        for row in reader: 
            for key in list(row):
                table[key].append(row[key])
            lines_read+=1

    print(table)
    print(lines_read)
    assert(lines_read == 3)
    
    roll = []
    pitch = []
    yaw = []

    for i in range(0, lines_read):
        rot = PyKDL.Rotation.RotX(0)
        rot.DoRotX(float(table['alpha'][i]))
        rot.DoRotZ(float(table['theta'][i]))
        rpy = rot.GetRPY()
        roll.append(rpy[0])
        pitch.append(rpy[1])
        yaw.append(rpy[2])

    x = []
    y = []
    z = []
    for i in range(0, lines_read):
	    x.append(float(table['a'][i]))
	    y.append(float(table['d'][i]) * math.sin(float(table['alpha'][i])))
	    z.append(float(table['d'][i]) * math.cos(float(table['alpha'][i])))

    with open(join('../config', config_file), 'w+') as f:
        for i in range(0, lines_read):
    	    f.write("roll%d: %.2f\n" % ((i+1), roll[i]))
    	    f.write("pitch%d: %.2f\n" % ((i+1), pitch[i]))
    	    f.write("yaw%d: %.2f\n" % ((i+1), yaw[i]))
    	    f.write("x%d: %.2f\n" % ((i+1), x[i]))
    	    f.write("y%d: %.2f\n" % ((i+1), y[i]))
    	    f.write("z%d: %.2f\n" % ((i+1), z[i]))

dh_2_rpy('dh_table.csv', 'urdf_params.yaml')