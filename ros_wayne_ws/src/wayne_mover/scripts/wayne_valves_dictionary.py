from ast import arg


VALVES_PINS = {
    "inflate":0,
    "deflate":1,
    "fll": 2,
    "frl":4,
    "mll":6,
    "mrl":8,
    "bll":10,
    "brl":10,
    "lb":12,
    "rb":14
}

def reset_valves_list():
    return (0,0,0,0,0,0,0,0,0,0,0,0,0,0)

def inflate (*args):
    new_valves_list = reset_valves_list()
    new_valves_list[VALVES_PINS['inflate']] = 1
    for a in args:
        new_valves_list[VALVES_PINS[a]] = 1
    return new_valves_list

def inflate_all ():
    new_valves_list = reset_valves_list()
    new_valves_list[VALVES_PINS['inflate']] = 1
    for a in range(2,15,2):
        new_valves_list[a] = 1
    return new_valves_list

def inflate_all_but (*args):
    new_valves_list = inflate_all()
    for a in args:
        new_valves_list[VALVES_PINS[a]] = 0
    return new_valves_list

def deflate (*args):
    new_valves_list = reset_valves_list()
    new_valves_list[VALVES_PINS['deflate']] = 1
    for a in args:
        new_valves_list[VALVES_PINS[a]+1] = 1
    return new_valves_list


def deflate_all ():
    new_valves_list = reset_valves_list()
    new_valves_list[VALVES_PINS['deflate']] = 1
    for a in range(3,15,2):
        new_valves_list[a] = 1
    return new_valves_list

def deflate_all_but (*args):
    new_valves_list = deflate_all()
    for a in args:
        new_valves_list[VALVES_PINS[a]+1] = 0
    return new_valves_list