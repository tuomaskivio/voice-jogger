#!/usr/bin/env python
# -*- coding: utf-8 -*-

import copy
from word2number import w2n

class CommandCreator(object):
	def __init__(self):

		# modes: STEP, DIRECTION AND DISTANCE
		self.mode = 'STEP'

		# step sizes: LOW, MEDIUM, HIGH
		self.step_size = 'MEDIUM'

		# original words from microphone_input after one speech
		self.original_words = []

		# current words. Amount of words decrease when buffering.
		self.current_words = []

		# do buffering when true
		self.buffering_ok = True

		self.all_words_lookup_table = {
			'start' : 'START',
			'started' : 'STARTED',
			'star' : 'START',
			'stat' : 'START',
			'part' : 'START',
			'stop' : 'STOP',
			'panda': 'PANDA',
			'ponder' : 'PANDA',
			'bond' : 'PANDA',
			'under' : 'PANDA',
			'fonda' : 'PANDA',
			'move' : 'MOVE',
			'moved' : 'MOVE',
			'more' : 'MOVE',
			'go' : 'MOVE',
			'up' : 'UP',
			'hop' : 'UP',
			'op' : 'UP',
			'down' : 'DOWN',
			"don't": 'DOWN',
			'left' : 'LEFT',
			'life' : 'LEFT',
			'write' : 'RIGHT',
			'right' : 'RIGHT',
			'alright' : 'RIGHT',
			'ride' : 'RIGHT',
			'forward': 'FORWARD',
			'front': 'FORWARD',
			'backward': 'BACKWARD',
			'back': 'BACKWARD',
			'for' : 'four',
			'to' : 'two',
			'then' : 'ten',
			'tree' : 'three',
			'tray' : 'three',
			'charity' : 'thirty',
			'mode' : 'MODE',
			'distance' : 'DISTANCE',
			'direction' : 'DIRECTION',
			'step' : 'STEP',
			'steps' : 'STEP',
			'low' : 'LOW',
			'glow' : 'LOW',
			'blow' : 'LOW',
			'flow' : 'LOW',
			'medium' : 'MEDIUM',
			'high' : 'HIGH',
			'hi' : 'HIGH',
			'size' : 'SIZE',
			'sized' : 'SIZE',
			'ice' : 'SIZE',
			'tool' : 'TOOL',
            'door' : 'TOOL',
            'dual' : 'TOOL',
            'tall' : 'TOOL',
            'all' : 'TOOL',
            'whole' : 'TOOL',
			'open' : 'OPEN',
			'close' : 'CLOSE',
            'glass' : 'CLOSE',
			'gloves' : 'CLOSE',
			'klaus' : 'CLOSE',
			'grasp' : 'GRASP',
			'rotate' : 'ROTATE',
			'list' : 'LIST',
			'lace' : 'LIST',
			'laced' : 'LIST',
			'least' : 'LIST',
			'shout' : 'SHOW',
			'show' : 'SHOW',
			'showed' : 'SHOW',
			'so' : 'SHOW',
			'task' : 'TASK',
			'tasks' : 'TASK',
			'desk' : 'TASK',
			'desks' : 'TASK',
			'dusk' : 'TASK',
			'play' : 'PLAY',
			'blade' : 'PLAY',
			'do' : 'DO',
  			'remove' : 'REMOVE',
  			'ramos' : 'REMOVE',
			'delete' : 'DELETE',
			'daily' : 'DELETE',
			'save' : 'SAVE',
			'saves' : 'SAVE',
			'say' : 'SAVE',
			'same' : 'SAVE',
			'home' : 'HOME',
			'tom' : 'HOME',
			'paul' : 'HOME',
			'finish' : 'FINISH',
			'vince' : 'FINISH',
			'phyllis' : 'FINISH',
			'phoenix' : 'FINISH',
			'record' : 'RECORD',
			'report' : 'RECORD',
			'gripper' : 'GRIPPER',
			'prepare' : 'GRIPPER',
			'grasp' : 'GRASP',
			'gasp' : 'GRASP',
			'position' : 'POSITION',
			'positions' : 'POSITION',
			'postseason' : 'POSITION',
			'spot' : 'SPOT',
			'spots' : 'SPOT',
			'other' : 'OTHER',
			'opposite' : 'OPPOSITE', 
			'counter' : 'COUNTER'
		}


	def getCommand(self, first_call):
		allwords = copy.copy(self.original_words)

		if first_call:
			self.buffering_ok = True
			# Filtering words
			filtered_words = []
			for word in self.original_words:
				# Check is word inside all_words_lookup_table
				checked_word = self.all_words_lookup_table.get(word)

				# Can't do buffering if one of these words exists
				if checked_word != None:
					if checked_word in ["MOVE", "ROTATE", "RECORD", "REMOVE", "DELETE", 
					"TASK", "DO", "PLAY", "SAVE", "POSITION", "SPOT"]:
						self.buffering_ok = False

				# Check is word number
				checked_number = self.get_number([word])

				# Can't do buffering if number exists
				if checked_number != None:
					self.buffering_ok = False

				# Add word to filtered_words
				# word is known word and number
				if checked_word != None and checked_number != None:
					filtered_words.append(word)
				# word is number
				elif checked_word == None and checked_number != None:
					filtered_words.append(word)
				# word is known word
				elif checked_word != None and checked_number == None:
					filtered_words.append(word)
				else:
					continue

			if self.buffering_ok:
				self.current_words = filtered_words
			else:
				self.current_words = []

			if self.original_words[0] != "":
				print(80*"-")
				print("All recorded words: ")
				print(self.original_words)
				print("")
				print("Filtered_words: ")
				print(filtered_words)
				print("")

		words = []
		if first_call:
			if self.buffering_ok:
				words = self.current_words
			else:
				words = self.original_words
		else:
			words = self.current_words

		# Take a new word from the words-list until word is found from all_words_lookup_table
		if len(words) > 0:
			command =  self.all_words_lookup_table.get(words.pop(0))
		else:
			return None

		while(command == None):
			if len(words) == 0:
				return None
			else:
				command = self.all_words_lookup_table.get(words.pop(0))
		
		if command == "START":
			return self.get_start_command(words)
		elif command == "STOP":
			return self.get_stop_command(words)
		elif command == "HOME":
			return ["HOME"]
		elif command == "MOVE":
			if self.mode == 'STEP':
				return self.get_move_command_step_mode(words)
			else:
				return self.get_move_command_direction_and_distance_mode(words)
		elif command == "MODE":
			return self.change_mode(words)
		elif command == "STEP":
			return self.change_step_size(words)
		elif command == "TOOL":
			return self.get_tool_command(words)

		#___________________MOVING COMMANDS____________________________
		elif command in ['UP', 'DOWN', 'LEFT', 'RIGHT', 'FORWARD', 'BACKWARD']:
			# Check distance
			if len(words) > 0:
				distance = self.get_number(words)
				print("Buffeeing: ", self.buffering_ok)
				if self.buffering_ok:
					return [command]
				elif not self.buffering_ok and distance != None:
					return [command, distance]
				else:
					print("Invalid moving command. ")
					return None
			return [command]

		#___________________ROTATE TOOL________________________________
		elif command == "ROTATE":
			if len(words) == 0:
				return ["ROTATE"]
			else:
				if self.all_words_lookup_table.get(words[0], '') in ['OTHER', 'COUNTER', 'OPPOSITE']:
					return ["ROTATE", "BACK"]
				else:
					print("Invalid " + command + " command.")
					return None

		#___________________RECORD TASKNAME_____________________________
		elif command == "RECORD":
			task_name = self.get_name(words)
			if task_name is not None:
				return ["RECORD", task_name]
			else:
				print('Invalid ' + command + ' command. Correct form: RECORD [task name]')
				return None

		#___________________REMOVE/DELETE TASK/POSITION______________________
		elif command == "REMOVE" or command == 'DELETE':
			# Remove position
			if self.all_words_lookup_table.get(words[0], '') in ['POSITION', 'SPOT']:
				words.pop(0)
				position_name = self.get_name(words)
				return ["REMOVE", "POSITION", position_name]
			else:
				task_name = self.get_name(words)
				if task_name is not None:
					return ["REMOVE", task_name]
				else:
					print('Invalid ' + command + ' command. Correct form: REMOVE/DELETE [task name]')
					return None

		#___________________PLAY/DO/TASK TASKNAME______________________
		elif command == 'TASK' or command == 'DO' or command == 'PLAY':
			task_name = self.get_name(words)
			if task_name is not None:
				return ["TASK", task_name]
			else:
				print('Invalid ' + command + ' command. Correct form: TASK/DO/PLAY [task name]')
				return None

		#___________________LIST/SHOW TASKS/POSITIONS__________________
		elif command == "LIST" or command == "SHOW":
			if len(words) != 1:
				print('Invalid command ' + command + '. Correct form: LIST/SHOW TASK/TASKS/POSITION/POSITIONS')
				return None
			else:
				# list tasks
				if self.all_words_lookup_table.get(words[0], '') in ['TASK']:
					return ['LIST', 'TASKS']
				elif self.all_words_lookup_table.get(words[0], '') in ['POSITION', 'SPOT']:
					return ['LIST', 'POSITIONS']
				else:
					print('Invalid command ' + words[0] + '. Correct form: LIST/SHOW TASK/TASKS/POSITION/POSITIONS')
					return None

		#___________________SAVE POSITION______________________________
		elif command == "SAVE":
			if self.all_words_lookup_table.get(words.pop(0), '') in ['POSITION', 'SPOT']:
				position_name = self.get_name(words)
				if position_name is not None:
					return ['SAVE', 'POSITION', position_name]
				else:
					print('Invalid ' + command + ' command. Correct form: SAVE POSITION/SPOT [position name]')
					return None
			else:
				print('Invalid ' + command + ' command. Correct form: SAVE POSITION/SPOT [position name]')
				return None

		#___________________MOVE TO POSITION___________________________
		elif command == "POSITION" or command == "SPOT":
			position_name = self.get_name(words)
			return ["POSITION", position_name]


		elif type(command) == str:
			# Command from all_words_lookup_table
			cmd = [command]
			for word in words:
				cmd.append(word)
			return cmd
		else:
			# The first word not in all_words_lookup_table
			return allwords

	def get_name(self, words):
		# no name
		if len(words) < 1:
			return None

		# only name. E.g. TASK
		elif len(words) == 1:
			# if name is only number
			name_is_number = self.get_number(words)
			if name_is_number is None:
				return words[0].upper()
			else:
				return name_is_number

		# number at the end of name. E.g. TASK1
		elif len(words) > 1:
			task_name = words.pop(0).upper()
			# Check does task_name has number
			number = self.get_number(words)
			if number is None:
				# Other word at the end of name. E.g. TASKPICK
				extra_name = ''.join(words)
				return task_name + extra_name
			else:
				# number at the end of name. E.g. TASK1
				task_name = task_name + str(number)
				return task_name


	def get_start_command(self, words):
	    try:
	        if len(words) != 1:
	            return None
	        robot_name = self.all_words_lookup_table.get(words.pop(0), '')
	        if robot_name not in ['PANDA']:
	            raise ValueError('Invalid robot name specified in start command')
	        
	        return ['START', robot_name]
	    
	    except Exception as e:
	        print('Invalid start command arguments received')
	        print(e)
	        return None

	def get_stop_command(self, words):
		try:
			if len(words) != 1:
				return None
			robot_name = self.all_words_lookup_table.get(words.pop(0), '')
			if robot_name not in ['PANDA']:
				raise ValueError('Invalid robot name specified in stop command')
			return ['STOP', robot_name]

		except Exception as e:
			print('Invalid stop command arguments received')
			print(e)
			return None

	def get_move_command_direction_and_distance_mode(self, words):
	    try:
	        if len(words) < 2:
	            return None
	        direction = self.all_words_lookup_table.get(words.pop(0), '')
	        if direction not in ['UP', 'DOWN', 'LEFT', 'RIGHT', 'FORWARD', 'BACKWARD']:
	            raise ValueError('Invalid direction specified in move command')
	        
	        value = self.get_number(words)
	        if value is None:
	            raise ValueError('Could not convert value to number in move command')
	        
	        return ['MOVE', direction, value]
	    
	    except Exception as e:
	        print('Invalid move command arguments received')
	        print(e)
	        return None

	def get_number(self, words):
		number_words = words.copy()
		# Replace words that sound like number with numbers
		for i,word in enumerate(words):
			new_word = self.all_words_lookup_table.get(word, None)
			if new_word:
				number_words[i] = new_word
		try:
			value = w2n.word_to_num(' '.join(number_words))
			return value
		except Exception as a:
			pass


	def get_move_command_step_mode(self, words):
		try:
			if len(words) < 1:
				return None

			direction = self.all_words_lookup_table.get(words.pop(0), '')
			if direction not in ['UP', 'DOWN', 'LEFT', 'RIGHT', 'FORWARD', 'BACKWARD']:
				raise ValueError('Invalid direction specified in move command')

			return ['MOVE', direction]

		except Exception as e:
			print('Invalid move command arguments received')
			print(e)
			return None

	def change_mode(self, words):
		try:
			if len(words) != 1:
				return None
			mode = self.all_words_lookup_table.get(words.pop(0), '')
			if mode not in ['STEP', 'DISTANCE']:
				raise ValueError('Mode: ', mode, ' not valid mode. Valid modes are STEP and DISTANCE.')

			self.mode = mode

			return ['MODE', mode]

		except Exception as e:
			print('Invalid mode change.')
			print(e)
			return None


	def change_step_size(self, words):
		try:
			if len(words) < 2:
				return None
			word2 = self.all_words_lookup_table.get(words.pop(0), '')
			if word2 != 'SIZE':
				raise ValueError('word2: ' + word2)
			size = self.all_words_lookup_table.get(words.pop(0), '')
			print("sizeee: ", size)
			if size not in ['LOW', 'MEDIUM', 'HIGH']:
				raise ValueError(word2)

			self.step_size = size

			return ['STEP', 'SIZE', size]

		except Exception as e:
			print('Invalid step size change. ', e)
			return None
        
	def get_tool_command(self, words):
		try:
			if len(words) != 1:
				return None
			word = words.pop(0)
			tool_state = self.all_words_lookup_table.get(word, '')
			if tool_state not in ['OPEN', 'CLOSE']:
				raise ValueError('Command: ', tool_state, 
					 ' not valid command for gripper tool. Valid commands are'
					  ' OPEN and CLOSE.')
            
			return ['TOOL', tool_state]
        
		except Exception as e:
			print('Invalid tool command. ', e)
			return None

		