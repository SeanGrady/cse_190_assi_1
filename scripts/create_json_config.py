import json

param_dict = {
    'pipe map': [['C','-','H','H','-'],
                ['C','-','H','-','-'],
                ['C','-','H','-','-'],
                ['C','C','H','H','H']],
    'texture map': [['S','S','S','S','R'],
                   ['R','R','S','R','R'],
                   ['R','S','S','S','S'],
                   ['S','R','R','S','R']],
    'move list': [[0,0],[0,1],[1,0],[1,0],[0,1],[0,1]],
    'starting pos': [3,3],
    'temp noise std dev': 10,
    'prob tex correct': 0.99,
    'uncertain motion': True,
    'prob move correct': 0.75,
    'seed': 0,
}

with open('parameters.json', 'w+') as infile:
    json.dump(param_dict, infile)
