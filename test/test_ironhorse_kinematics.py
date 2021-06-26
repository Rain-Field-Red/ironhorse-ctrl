
import numpy
import timee

import pyrbdl as rbdl

urdf = '../models/iron_horse/iron_horse_simple.urdf'
#robot, collision_model, visual_model = pin.buildModelsFromUrdf(URDF)
rbdl.buildModelsFromUrdf(urdf)

print(robot.model)
print(robot.model.nq)


