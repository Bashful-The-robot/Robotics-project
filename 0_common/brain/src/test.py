#%%
import py_trees as pt
import rospy
class SayHello(pt.behaviour.Behaviour):
    def __init__(self):
        super(SayHello, self).__init__()

    def update(self):
        print("Hello!")
        return pt.common.Status.FAILURE

class SayGoodbye(pt.behaviour.Behaviour):
    def __init__(self):
        super(SayGoodbye, self).__init__()

    def update(self):
        print("Goodbye!")
        return pt.common.Status.FAILURE

root = pt.composites.Selector()
root.add_children([SayHello(), SayGoodbye()])

tree = pt.trees.BehaviourTree(root)

# # execute the behaviour tree
tree.setup(timeout=2)
# for i in range(10):
#     print(i)
tree.tick_tock(1)