f = open('testfile',"wb")
import os
f.write(os.urandom(255))
f.close()
os.stat("testfile").st_size
