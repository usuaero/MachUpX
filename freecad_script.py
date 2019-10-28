import sys
sys.settrace = True
sys.path.append('/usr/lib/freecad/lib')

from FreeCAD import Base
import Part

p = Base.Vector(1,0,0)
s = Part.makeBox(1,1,1,p)

s.exportStep("box.stp")