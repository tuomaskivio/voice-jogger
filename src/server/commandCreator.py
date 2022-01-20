#!/usr/bin/env python
# -*- coding: utf-8 -*-

from word2number import w2n

class CommandCreator(object):
	def __init__(self):

		# modes: STEP, DIRECTION AND DISTANCE
		self.mode = 'STEP'

		# step sizes: LOW, MEDIUM, HIGH
		self.step_size = 'MEDIUM'

		self.all_words_lookup_table = {
			'start' : 'START',
			'star' : 'START',
			'stat' : 'START',
			'stop' : 'STOP',
			'panda': 'PANDA',
			'ponder' : 'PANDA',
			'move' : 'MOVE',
			'moved' : 'MOVE',
			'more' : 'MOVE',
			'go' : 'MOVE',
			'up' : 'UP',
			'down' : 'DOWN',
			"don't": 'DOWN',
			'left' : 'LEFT',
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
			'mode' : 'MODE',
			'distance' : 'DISTANCE',
			'direction' : 'DIRECTION',
			'step' : 'STEP',
			'steps' : 'STEP',
			'low' : 'LOW',
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
			'list' : 'LIST',
			'least' : 'LIST',
			'shout' : 'SHOW',
			'show' : 'SHOW',
			'showed' : 'SHOW',
			'so' : 'SHOW',
			'task' : 'TASK',
			'tasks' : 'TASK',
			'remove' : 'REMOVE',
			'delete' : 'DELETE',
			'daily' : 'DELETE'
		}


	def getCommand(self, words):
		# first word tells what we want to do
		word1st = words.pop(0)
		command = self.all_words_lookup_table.get(word1st)
		
		if command == "START":
			return self.get_start_command(words)
		elif command == "STOP":
			return self.get_stop_command(words)
		elif command == "HOME":
			return "HOME"
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

		#___________________LIST/SHOW TASKS______________________
		elif command == "LIST" or command == "SHOW":
			if len(words) != 1:
				return None
			else:
				if self.all_words_lookup_table.get(words[0], '') not in ['TASK']:
					print('Invalid command ' + words[0] + '. Did you mean LIST TASK?')
					return None
				else:
					return ['LIST', 'TASKS']

		#___________________REMOVE TASKNAME______________________
		elif command == "REMOVE" or command == 'DELETE': 
			if len(words) < 2:
				return None
			# TODO

		else:
			return None


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
	        
	        number_words = words.copy()
	        # Replace words that sound like number with numbers
	        for i,word in enumerate(words):
	            new_word = self.all_words_lookup_table.get(word, None)
	            if new_word:
	                number_words[i] = new_word
	                
	        value = w2n.word_to_num(' '.join(number_words))
	        if value is None:
	            raise ValueError('Could not convert value to number in move command')
	        
	        return ['MOVE', direction, value]
	    
	    except Exception as e:
	        print('Invalid move command arguments received')
	        print(e)
	        return None

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

		