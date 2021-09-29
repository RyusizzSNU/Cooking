import csv
import os

def get_all_recipes():
    recipes = []
    for fname in os.listdir('recipes'):
        tokens = fname.split('.')
        if len(tokens) >= 2 and tokens[-1] == 'csv':
            recipes.append(tokens[0])
    return recipes


def read_recipe(name):
    with open('recipes/%s.csv'%name, 'r') as f:
        instructions = []
        first = True
        for line in csv.reader(f):
            if first:
                first = False
                continue
            instructions.append([line[9], line[11]])
        return instructions
