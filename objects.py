import sched
import datetime, time
import math

class Periodic_Scheduler(object):

	def __init__(self, input_ob):
		self.scheduler = sched.scheduler(time.time, time.sleep)
		self.input_ob = input_ob

		# self.n_events = 0

	def setup(self, interval, action, actionargs = ()):
		action(*actionargs)

		for i in range(self.input_ob.n_reps):
			# print ("scheduled")
			# print (interval)
			# print (i * interval)
			self.scheduler.enter(i * interval, 1, self.action_program, (action, actionargs))

	def action_program(self, action, actionargs = ()):
		action(*actionargs)

	def run(self):
		# print ("run")
		self.scheduler.run()

class Operation_Input(object):

	def __init__(self, n_vials, steps, time, n_reps):
		self.n_vials = n_vials
		self.steps = steps
		self.time = time
		self.n_reps = n_reps
		#self.store_path = store_path

		self.max_op_len = 0.0 # max operation length float value in minutes

	def set_max_op_len(self, input):
		self.max_op_len = input

