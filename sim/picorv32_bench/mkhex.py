import struct,sys
d=open(sys.argv[1],"rb").read(); d+=b"\0"*((-len(d))%4)
open(sys.argv[2],"w").write("\n".join("%08x"%w for w in struct.unpack("<%dI"%(len(d)//4),d))+"\n")
