import os
import sys

def readFile(str_path):
    time = []
    x = []
    y = []
    z = []
    qx = []
    qy = []
    qz = []
    qw = []

    fin = open(str_path,'r')
    fin.readline()
    line = fin.readline()
    while line:
        parts = line.split(",")
        time.append(float(parts[0]))
        x.append(float(parts[1]))
        y.append(float(parts[2]))
        z.append(float(parts[3]))
        qw.append(float(parts[4]))
        qx.append(float(parts[5]))
        qy.append(float(parts[6]))
        qz.append(float(parts[7]))
        line = fin.readline()
    return time,x,y,z,qx,qy,qz,qw

file_path = sys.argv[1]
t,x,y,z,qx,qy,qz,qw = readFile(file_path)


fout = open("groundtruth.txt",'w')
for i in range(len(t)):
    line_str = str(t[i])+" "+str(x[i])+" "+str(y[i])+" "+str(z[i])+" "+str(qx[i])+" "+str(qy[i])+" "+str(qz[i])+" "+str(qw[i])+"\n"
    fout.write(line_str)
fout.close()