# import os, sys, inspect, io
#
# cmd_folder = os.path.realpath(
# os.path.dirname(
#     os.path.abspath(os.path.split(inspect.getfile(inspect.currentframe()))[0])))
# print cmd_folder
# print sys.path
#
# if cmd_folder not in sys.path:
#     sys.path.insert(0, cmd_folder)
#
# from transitions.extensions import MachineFactory
# from IPython.display import Image, display, display_png
#
#
# class Matter(object):
#     def is_hot(self):
#         return True
#
#     def is_too_hot(self):
#         return False
#
#     def show_graph(self, **kwargs):
#         # print(self.get_graph(**kwargs).string())
#         stream = io.BytesIO()
#         self.get_graph(**kwargs).draw(stream, prog='dot', format='png')
#         display(Image(stream.getvalue()))
#
#
# GraphMachine = MachineFactory.get_predefined(graph=True, nested=True)
# states = ['standing', 'walking', {'name': 'caffeinated', 'children':['dithering', 'running']}]
# transitions = [
#   ['walk', 'standing', 'walking'],
#   ['go', 'standing', 'walking'],
#   ['stop', 'walking', 'standing'],
#   {'trigger': 'drink', 'source': '*', 'dest': 'caffeinated_dithering',
#    'conditions':'is_hot', 'unless': 'is_too_hot'},
#   ['walk', 'caffeinated_dithering', 'caffeinated_running'],
#   ['relax', 'caffeinated', 'standing'],
#   ['sip', 'standing', 'caffeinated']
# ]
#
# model = Matter()
# machine = GraphMachine(model=model,
#                        states=states,
#                        transitions=transitions,
#                        auto_transitions=False,
#                        initial='standing',
#                        title="Mood Matrix",
#                        show_conditions=True)
# # model.walk()
# model.show_graph()
#

from transitions import Machine,State
import random
# Our old Matter class, now with  a couple of new methods we
# can trigger when entering or exit states.
class Matter(object):
    def say_hello(self): print("hello, new state!")
    def say_goodbye(self): print("goodbye, old state!")

lump = Matter()
# Same states as above, but now we give StateA an exit callback
states = [
    State(name='solid', on_exit=['say_goodbye']),
    'liquid',
    { 'name': 'gas' }
    ]

machine = Machine(lump, states=states)
machine.add_transition('sublimate', 'solid', 'gas')

# Callbacks can also be added after initialization using
# the dynamically added on_enter_ and on_exit_ methods.
# Note that the initial call to add the callback is made
# on the Machine and not on the model.
# machine.on_enter_gas('say_hello')
machine.on_enter_solid('say_hello')
# Test out the callbacks...
machine.set_state('solid')
lump.sublimate()