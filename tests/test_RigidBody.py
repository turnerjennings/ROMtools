from src.ROMtools.RigidBody import *

class TestRigidBody:
    #check init class for rigid body
    def test_init_assert(self):
        #assert all values stored correctly
        
        obj = RigidBody(ID=0, m=1, R=2, name="object")
        assert obj.ID == 0
        assert obj.m == 1
        assert obj.R == 2
        assert obj.name == "object"
        assert obj.x0 == 0.0
        assert obj.y0 == 0.0
        assert obj.t0 == 0.0
        assert obj.I == 4.0
        assert obj.p0 == (0.0,0.0,0.0)
        assert obj.v0 == (0.0,0.0,0.0)
        assert obj.a0 == (0.0,0.0,0.0)
        