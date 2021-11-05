from word2number import w2n


def get_move_command(words):
    try:
        if len(words) < 2:
            return None
        direction = CMD_ARGS_LOOKUP_TABLE.get(words.pop(0), '')
        if direction not in ['UP', 'DOWN', 'LEFT', 'RIGHT', 'FORWARD', 'BACKWARD']:
            raise ValueError('Invalid direction specified in move command')
        
        number_words = words.copy()
        # Replace words that sound like number with numbers
        for i,word in enumerate(words):
            new_word = CMD_ARGS_LOOKUP_TABLE.get(word, None)
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
        
        
def get_start_command(words):
    try:
        if len(words) != 1:
            return None
        robot_name = CMD_ARGS_LOOKUP_TABLE.get(words.pop(0), '')
        if robot_name not in ['PANDA']:
            raise ValueError('Invalid robot name specified in start command')
        
        return ['START', robot_name]
    
    except Exception as e:
        print('Invalid start command arguments received')
        print(e)
        return None
        

def get_stop_command(words):
    try:
        if len(words) != 1:
            return None
        robot_name = CMD_ARGS_LOOKUP_TABLE.get(words.pop(0), '')
        if robot_name not in ['PANDA']:
            raise ValueError('Invalid robot name specified in stop command')
        
        return ['STOP', robot_name]
    
    except Exception as e:
        print('Invalid stop command arguments received')
        print(e)
        return None
            
    
CMD_ARGS_LOOKUP_TABLE = {
'start' : 'START',
'stop' : 'STOP',
'panda': 'PANDA',
'move' : 'MOVE',
'moved' : 'MOVE',
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
}


COMMANDS = {
'START': get_start_command,
'STOP': get_stop_command,
'MOVE': get_move_command,
}


