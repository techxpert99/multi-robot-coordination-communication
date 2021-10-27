from random import randint,random

def choose(minp,maxp):
    return minp+random()*(maxp-minp)

def choose_obstacle(type,side_bounds,radius_bounds,bounds):
    minx,miny,maxx,maxy = bounds
    mins,maxs = side_bounds
    minr,maxr = radius_bounds
    
    center = (choose(minx,maxx),choose(miny,maxy))
    if type == 'cube':
        side = choose(mins,maxs)
        return ('cube',center,side)
    elif type == 'cylinder' or type == 'sphere':
        radius = choose(minr,maxr)
        return (type,center,radius)

def obstacle_bounds(obstacle):
    t,(x,y),d = obstacle
    if t == 'cube':
        b = (x-d/2,y-d/2,x+d/2,y+d/2)
    elif t in ['cylinder','sphere']:
        b = (x-d,y-d,x+d,y+d)
    return b

def do_spawn(obstacle,obstacles,arena_bounds):
    b = obstacle_bounds(obstacle)
    if b[0] < arena_bounds[0] or b[1] < arena_bounds[1] or b[2] > arena_bounds[2] or b[3] > arena_bounds[3]: return False
    for obstacle in obstacles:
        p,q,r,s = obstacle_bounds(obstacle)
        if p < b[0] or q < b[1] or r > b[2] or s > b[3]:
            pass
        else:
            return False
    return True

def spawn_obstacles(num_obstacles,side_bounds,radius_bounds,arena_bounds,num_tries):
    obstacles = []
    for typ,index in [('cube',0),('cylinder',1),('sphere',2)]:
        for i in range(num_obstacles[index]):
            for j in range(num_tries):
                obstacle = choose_obstacle(typ,side_bounds,radius_bounds,arena_bounds)
                if do_spawn(obstacle,obstacles,arena_bounds):
                    obstacles.append(obstacle)
                    break       
    return obstacles