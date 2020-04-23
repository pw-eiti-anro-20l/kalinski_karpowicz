#! /usr/bin/python

# funkcja, read_DH(path_to_DH_table), tworzy plik params.yaml w folderze config i zwraca do niego sciezke.
from tf.transformations import translation_matrix, rotation_matrix, \
     concatenate_matrices, euler_from_matrix, translation_from_matrix
from os.path import join, dirname
import csv
import math
import rospy

table = {'i':[], 'a':[], 'd':[], 'alpha':[], 'theta':[]}

def dh_2_rpy(table_name, config_file):
    lines_read = 0 #nie bierzemy pod uwage 1 wiersza

    with open(join('config', table_name), 'r') as f:
        reader = csv.DictReader(f, delimiter=' ')
        for row in reader: 
            for key in list(row):
                table[key].append(float(row[key]))
            lines_read+=1

    print(table)
    print(lines_read)
    assert(lines_read == 3)
    
    roll = []
    pitch = []
    yaw = []

    xes = []
    yes = []
    zes = []

    x_axis = (1, 0, 0)
    z_axis = (0, 0, 1)
    params = []

    for i in range(0, lines_read):
        tz = translation_matrix((0, 0, table['d'][i]))  
        rz = rotation_matrix(table['theta'][i], z_axis)    
        tx = translation_matrix((table['a'][i], 0, 0))  
        rx = rotation_matrix(table['alpha'][i], x_axis) 

        params.append({'theta': table['theta'][i], 'alpha': table['alpha'][i], 'a': table['a'][i], 'd': table['d'][i]})

        #params = table

        dh_matrix = concatenate_matrices(tz, rz, tx, rx)

        (r, p, y) = euler_from_matrix(dh_matrix)
        (x_, y_, z_) = translation_from_matrix(dh_matrix)

        roll.append(r)
        pitch.append(p)
        yaw.append(y)
        xes.append(x_)
        yes.append(y_)
        zes.append(z_)

    #params = []

    with open(join('config', config_file), 'w+') as f:
        for i in range(0, lines_read):
            #joint = {}
    	    f.write("roll%d: %.2f\n" % ((i+1), roll[i]))
            #joint['roll'] = float(roll[i])

    	    f.write("pitch%d: %.2f\n" % ((i+1), pitch[i]))
            #joint['pitch'] = float(pitch[i])

    	    f.write("yaw%d: %.2f\n" % ((i+1), yaw[i]))
            #joint['yaw'] = float(yaw[i])

    	    f.write("x%d: %.2f\n" % ((i+1), xes[i]))
            #joint['x'] = float(xes[i])

    	    f.write("y%d: %.2f\n" % ((i+1), yes[i]))
            #joint['y'] = float(yes[i])

    	    f.write("z%d: %.2f\n" % ((i+1), zes[i]))
            #joint['z'] = float(zes[i])

            #params.append(joint)

    return params 

if __name__ == '__main__':
    rospy.init_node("gazebo_odometry_node")

    table_path = rospy.get_param('~dh_table_path', 'dh_table.csv')
    params = dh_2_rpy(table_path, join(dirname(table_path), 'urdf_params.yaml'))
    rospy.loginfo('table converted')
    rospy.loginfo('setting parameters')
    rospy.set_param('robot_params', params)
    rospy.loginfo('params sent to parameter server')