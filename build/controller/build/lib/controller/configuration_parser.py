def read_conf(conf_file='/home/ghost/Desktop/dev_ws/src/controller/controller/world.conf'):
    def parse(line):
        try:
            key,line = ( _.strip() for _ in line.split('(',1))
            type,line = ( _.strip() for _ in line.split(')',1))
            line = line.split(':',1)[-1]
            if type == 'int':
                converter = int
            elif type == 'float':
                converter = float
            else:
                converter = str
            vec = []
            for dim in line.split(','):
                vec.append(converter(dim.strip()))
            if len(vec) > 1:
                return key,tuple(vec)
            elif len(vec) == 1:
                return key,vec[0]
            else:
                return key,''
        except:
            return None,None
    
    confd = dict()
    with open(conf_file,'r') as f:
        for l in f.readlines():
            k,v = parse(l)
            confd[k] = v
    return confd