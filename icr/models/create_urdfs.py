import sys

# package name
pname = sys.argv[1]

# read colors
colorlist = []
colist = open("colors.txt","r")
for coline in colist:
    colorlist.append(coline.replace("material Gazebo/","").replace("\n",""))
print colorlist
colist.close()

# read model names
stllist = []
fstlist = open("stllist.txt","r")
for fname in fstlist:
    stllist.append(fname.replace(".stl","").replace("\n",""))
print stllist
fstlist.close()
        
        
        
for c in colorlist:
    for m in stllist:
        t = open("TEMPLATE.urdf","r")
        f = open("urdf/"+c+"_"+m+".urdf","w")
        for line in t:
            line = line.replace("${MODEL_ID}",m)
            line = line.replace("${COLOR}",c)
            line = line.replace("${PACKAGE}",pname)
            print >>f, line,
        print "Did\t\t"+c+"\t\t"+m
        f.close()
        t.close()

