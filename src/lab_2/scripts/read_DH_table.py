# funkcja, read_DH(path_to_DH_table), tworzy plik params.yaml w folderze config i zwraca do niego sciezke.
from os.path import join
import pyyaml
import itertools
import csv

table = {'i':[], 'a':[], 'd':[], 'alfa':[], 'theta':[]}

def dh_2_rpy(table_name):
    with open(join('../config', table_name), 'r') as f:
        reader = csv.DictReader(f, dialect=csv.unix_dialect)
        for row in reader: 
            for key in list(row):
                table[key].append(row[key])

    print(table)


        
dh_2_rpy('dh_table.csv')