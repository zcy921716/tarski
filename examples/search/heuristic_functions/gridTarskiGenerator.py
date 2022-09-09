
import tarski
import random
from tarski.syntax import land, Atom
import tarski.fstrips as fs
import tarski.io.fstrips as fsio
from tarski.io import FstripsWriter as fw
from tarski.syntax import Formula


###
# Create a language for the gridworld
# Parameters:
# x: length for the gridworld
# y: height for the gridworld
###

def gridworldGenerator(x, y):
    lang = fs.language("blocksworld")

    ats = [lang.predicate(f'at_{i}_{j}') for i in range(x) for j in range(y)]
    visiteds = [lang.predicate(f'visited_{i}_{j}') for i in range(x) for j in range(y)]

    connecteds = []
    for i in range(x):
        for j in range(y - 1):
            connecteds.append(lang.predicate(f'connected_{i}_{j}_{i}_{j + 1}'))
            connecteds.append(lang.predicate(f'connected_{i}_{j + 1}_{i}_{j}'))

    for i in range(x - 1):
        for j in range(y):
            connecteds.append(lang.predicate(f'connected_{i}_{j}_{i + 1}_{j}'))
            connecteds.append(lang.predicate(f'connected_{i + 1}_{j}_{i}_{j}'))

    return lang, ats, visiteds, connecteds


def coordinate(predicate):
    return str(predicate.symbol).split('_')[1:]


###
# Create a problem for the gridworld
# Parameters:
# lang: language of the gridworld
#
# problemName: name of the generated problem
# (optional) location: initial location of the agent eg: a tuple (5, 5)
# (optional) goals: set of goals (a list of locations)
# (optional) numOfGoals: number of goals (default: 5)
###

def gridworldProblemGenerator(lang, ats, visiteds, connecteds, problemName, location=None, goals=None, numOfGoals=5):
    problem = fs.create_fstrips_problem(domain_name='gridworld', problem_name=problemName, language=lang)

    init = tarski.model.create(lang)

    if not location:
        location = random.choice(ats)  # randomly choose a location as the start location

    init.add(location())
    x, y = coordinate(location)

    for visited in visiteds:
        if coordinate(visited) == [x, y]:
            init.add(visited())
            break

    # Add adjacent information
    for atom in connecteds:
        init.add(atom())

    problem.init = init

    if not goals:
        goals = random.sample(visiteds, numOfGoals)  # randomly choose some locations

    problem.goal = land(goals[0]())

    for goal in goals[1:]:
        problem.goal = goal() & problem.goal

    # move action
    for connected in connecteds:
        x0, y0, x1, y1 = str(connected.symbol).split('_')[1:]
        for at in ats:
            if coordinate(at) == [x0, y0]:
                at0 = at
            if coordinate(at) == [x1, y1]:
                at1 = at

        for visited in visiteds:
            if coordinate(visited) == [x1, y1]:
                visited1 = visited

        mv = problem.action(f'move_{x0}_{y0}_{x1}_{y1}', [],
                            precondition=at0() & connected(),
                            effects=[
                                fs.DelEffect(at0()),
                                fs.AddEffect(at1()),
                                fs.AddEffect(visited1())
                            ])

    return problem

def write_to_pddl( problem, domain_file = 'gridworld.pddl', instance_file = 'test.pddl' ):

    # NIR: Write to PDDL from Tarski
    writer = fw(problem)
    writer.write(domain_file, instance_file)
