from time import time
from controller.wrappers import NodeWrapper, ThreadWrapper
from robot_communication_interfaces.msg import CommunicationMessage
from controller.auxiliary import wait

class CoCoNode:
    
    def __init__(self,store):
        self._int = None
        self.store = store
        self.store.set('estimator','position',(0.0,0.0,0.0))
        self.id = store.get('names','namespace')
        self.desired_team_definition = {('leader',1,0.9),('mapper',2,0.1)}
        self.role_suitabilities = {'leader':1.0,'mapper':0.1}
        self.suitabilities = {(self.id,role,suitability) for role,suitability in self.role_suitabilities.items()}
        self.team = {(self.id,'leader',self.role_suitabilities['leader'])}
        self.MAX_EXCHANGES_IN_TRANSIT = 2**32
        self.team_change_message = None
        self.last_contact = dict()
        self.exchange_number = 0
        self.exchange_period = 50 #store.get('','')
        self.exchange_timeout = 10 #store.get('','')
        self.teammate_lost_timeout = 30 #store.geT('','')
        self.rejected_exchanges = set()
        self.robot_lost_timeout = 40
        self.rejection_timeout = 20
        self.exchange_inbox = []
        self.last_exchange = dict()
        self.ongoing_exchange = None
        self.can_exchange = True
        store.set('coco','inbox',[])
        self.exchange_set = set()
        self.node = NodeWrapper(store.get('names','coco'),store)
        self.node.create_subscription('com.in',self.on_receipt_of_message)
        self.publisher = self.node.create_publisher('broadcasts')
        self.thread = ThreadWrapper(self.callback,1)
        self.thread.start()

    def destroy(self):
        self.thread.stop()
        self.node.destroy()

    def on_receipt_of_message(self,msg: CommunicationMessage):
        self.store.lock('coco')
        inbox = self.store.unsafe_get('coco','inbox')
        inbox.append(msg)
        self.store.release('coco')

    
    def send_message_without_data(self,type,receiver=None,exchange_number=None):
        if not self.store.has('estimator','position'): return False
        msg = CommunicationMessage()
        msg.robot_id = self.id
        if receiver is not None:
            msg.receiver_robot_id = receiver
        msg.message_type = type
        if exchange_number is not None:
            msg.exchange_number = exchange_number
        msg.position = self.store.get('estimator','position')
        self.publisher.publish(msg)
        # if type != 'discovery':
        #     if exchange_number is None:
        #         print('SEND',type,':',self.id,'->',receiver,f'{time()%10000:.1f}')
        #     else:
        #         print('SEND',type,':',self.id,'->',receiver,f'{time()%10000:.1f}','seq=',exchange_number)
        return True
    
    def _dict_to_list(self,d):
        k,v = [],[]
        for a in d:
            k.append(a)
            v.append(d[a])
        return k,v

    def send_exchange_message(self,receiver,exchange_id):
        if not self.store.has('estimator','position'): return False
        msg = CommunicationMessage()
        msg.robot_id = self.id
        msg.message_type = 'ack'
        msg.exchange_number = exchange_id
        msg.position = self.store.get('estimator','position')
        msg.receiver_robot_id = receiver
        su_id,su_rol,su_val = list(),list(),list()
        for id,rol,val in self.suitabilities:
            su_id.append(id)
            su_rol.append(rol)
            su_val.append(val)
        msg.suitability_robots = su_id
        msg.suitability_roles = su_rol
        msg.suitability_values = su_val
        self.publisher.publish(msg)
        # print('SEND','ack',':',self.id,'->',receiver,f'{time()%10000:.1f}','seq=',exchange_id)
        return True

    def process_inbox(self):
        self.store.lock('coco')
        inbox = self.store.unsafe_get('coco','inbox')
        for message in inbox:
            if message.message_type == 'discovery':
                self.last_contact[message.robot_id] = time()
            elif message.message_type in {'exchange','ack','wait','reject','fin'}:
                #print('RECEIVE',message.message_type,':',message.robot_id,'->',self.id,f'{time()%10000:.1f}','seq=',message.exchange_number)
                self.exchange_inbox.append(message)
            elif message.message_type in {'team_change'}:
                self.team_change_message = message
        self.store.unsafe_set('coco','inbox',[])
        self.store.release('coco')
    
    def trim_team(self):
        new_team = set()
        removed_members = set()
        for id,role,suitability in self.team:
            if id != self.id:
                if id in self.last_contact and (time()-self.last_contact[id] < self.teammate_lost_timeout):
                    new_team.add((id,role,suitability))
                else:
                    removed_members.add(id)
            else:
                new_team.add((id,role,suitability))
        if len(removed_members) > 0:  
            suitabilities = set(self.suitabilities)
            for id,rol,su in suitabilities:
                if id in removed_members:
                    self.suitabilities.remove((id,rol,su))
            self.team = new_team

    def trim_last_contacted(self):
        for id in dict(self.last_contact):
            if time()-self.last_contact[id] >= self.robot_lost_timeout:
                del self.last_contact[id]
                if id in self.last_exchange:
                    del self.last_exchange[id]
                if id in self.exchange_set:
                    self.exchange_set.remove(id)
    
    def trim_rejected_exchanges(self):
        for id,t in set(self.rejected_exchanges):
            if time()-t >= self.rejection_timeout:
                self.rejected_exchanges.remove((id,t))

    def select_team_member(self,role,robot_set:set):
        member = None
        for id,rol,su in robot_set:
            if member is None and rol == role: member = (id,role,su)
            elif rol == role and (su,id) > (member[2],member[0]): member = (id,role,su)
        obsolete_roles = set()
        for robot in robot_set:
            if robot[0] == member[0]:
                obsolete_roles.add(robot)
        return member,robot_set-obsolete_roles
    
    def select_extra_team_member(self,robot_set:set):
        member = None
        for id,rol,su in robot_set:
            if rol == 'leader': continue
            if member is None: member = (id,rol,su)
            elif (su,id) > (member[2],member[0]): member = (id,rol,su)
        obsolete_roles = set()
        for robot in robot_set:
            if robot[0] == member[0]:
                obsolete_roles.add(robot)
        return member,robot_set-obsolete_roles
    
    def create_team_without_extra(self,team_definition,robot_set):
        team_def_lst = list(team_definition)
        robot_def_set = set(robot_set)
        team_def_lst.sort(key=lambda role: role[2], reverse= True)
        final_team = set()
        for team_role,num_members,weightage in team_def_lst:
            num_selected = 0
            while num_selected < num_members and len(robot_def_set) > 0:
                member,robot_def_set = self.select_team_member(team_role,robot_def_set)
                final_team.add(member)
                num_selected += 1
            if len(robot_def_set) == 0:
                return final_team,robot_def_set
        return final_team,robot_def_set
    
    def create_teams(self,team_definition,robot_set):
        teams = list()
        while len(robot_set) > 0:
            team,robot_set = self.create_team_without_extra(team_definition,robot_set)
            teams.append(team)
        return teams

    def stabilize_teams(self,message: CommunicationMessage):
        
        su_robots = message.suitability_robots
        su_roles = message.suitability_roles
        su_values = message.suitability_values

        for i,id in enumerate(su_robots):
            if id == message.robot_id and su_roles[i] == 'leader':
                peer_leader_suitability = su_values[i]
                break
        
        if (peer_leader_suitability,id) > (self.role_suitabilities['leader'],self.id):
            return None
        
        robot_set_2 = {(su_robots[i],su_roles[i],su_values[i]) for i in range(len(su_robots))}
        robot_set_1 = self.suitabilities

        robot_set = robot_set_2.union(robot_set_1)

        teams = self.create_teams(self.desired_team_definition,robot_set)
        
        _tid,_id,_rol,_su = list(),list(),list(),list()

        for tid,team in enumerate(teams):
            for id,rol,su in team:
                _tid.append(tid)
                _id.append(id)
                _rol.append(rol)
                _su.append(su)
        
        sid,srol,sval = list(),list(),list()
        for id,rol,val in robot_set:
            sid.append(id)
            srol.append(rol)
            sval.append(val)

        return (_tid,_id,_rol,_su),(sid,srol,sval)

    def send_finish_exchange_message(self,peer,teamset,exchange_id):
        (tid,id,rol,su),(sid,srol,sval) = teamset
        msg = CommunicationMessage()
        msg.message_type = 'fin'
        msg.position = self.store.get('estimator','position')
        msg.robot_id = self.id
        msg.receiver_robot_id = peer
        msg.teams_tid = tid
        msg.teams_id = id
        msg.exchange_number = exchange_id
        msg.teams_rol = rol
        msg.teams_su = su
        msg.suitability_robots = sid
        msg.suitability_roles = srol
        msg.suitability_values = sval
        self.publisher.publish(msg)
        # print('SEND','fin',':',self.id,'->',peer,f'{time()%10000:.1f}','seq=',exchange_id)
    
    def send_team_change_message(self,peer,team,suitabilities):
        msg = CommunicationMessage()
        msg.message_type = 'team_change'
        msg.position = self.store.get('estimator','position')
        msg.robot_id = self.id
        sval,srol,sid = list(),list(),list()
        id,rol,su = list(),list(),list()
        for _id,_rol,_su in team:
            id.append(_id)
            rol.append(_rol)
            su.append(_su)
        for _srol,_sid,_sval in suitabilities:
            sval.append(_sval)
            srol.append(_srol)
            sid.append(_sid)
        msg.suitability_robots = sid
        msg.suitability_roles = srol
        msg.suitability_values = sval
        msg.teams_id = id
        msg.teams_rol = rol
        msg.teams_su = su
        self.publisher.publish(msg)
        # print('SEND','team_change',':',self.id,'->',peer,f'{time()%10000:.1f}')


    def finish_exchange(self,teamset):
        (tid,id,rol,su),(sid,srol,sval) = teamset
        teams = dict()
        suitabilities = dict()
        id_tid_map = dict()
        for i in range(len(tid)):
            _tid,_id,_rol,_su = tid[i],id[i],rol[i],su[i]
            if _tid not in teams:
                teams[_tid] = set()
                suitabilities[_tid] = set()
            teams[_tid].add((_id,_rol,_su))
            id_tid_map[_id] = _tid
        for i in range(len(sid)):
            _sid,_srol,_sval = sid[i],srol[i],sval[i]
            suitabilities[id_tid_map[_sid]].add((_sid,_srol,_sval))
        for id,_,_ in self.team:
            if id != self.id:
                self.send_team_change_message(id,teams[id_tid_map[id]],suitabilities[id_tid_map[id]])
        self.team = teams[id_tid_map[self.id]]
        self.suitabilities = suitabilities[id_tid_map[self.id]]

    def exchange_protocol(self):

        for robot in self.last_contact:
            if robot < self.id and robot not in self.rejected_exchanges and (robot not in self.last_exchange or time()-self.last_exchange[robot] >= self.exchange_period):
                self.exchange_set.add(robot)
        
        received = False
        if len(self.exchange_inbox) > 0:
            received = True
            message = self.exchange_inbox.pop()
            typ,id,eid = message.message_type,message.robot_id,message.exchange_number
            if typ == 'exchange':
                if self.ongoing_exchange is not None:
                    if self.ongoing_exchange[2] == eid:
                        if id == self.ongoing_exchange[0]:
                            self.send_exchange_message(id,eid)
                        elif not self.can_exchange:
                            self.send_message_without_data('reject',id,eid)
                        elif self.ongoing_exchange is not None:
                            self.send_message_without_data('wait',id,eid)
                        elif id in self.last_exchange and time()-self.last_exchange[id]<self.exchange_period:
                            self.send_message_without_data('reject',id,eid)
                    else:
                        return
                else:
                    if id in self.last_exchange and time()-self.last_exchange[id]<self.exchange_period:
                        self.send_message_without_data('reject',id,eid)
                    else:
                        self.ongoing_exchange = (id,time(),eid)
                        self.send_exchange_message(id,eid)
            elif not self.ongoing_exchange or id != self.ongoing_exchange[0]:
                return
        
        if self.ongoing_exchange is None and self.can_exchange and len(self.exchange_set) > 0:
            self.exchange_number = (self.exchange_number+1)%self.MAX_EXCHANGES_IN_TRANSIT
            self.ongoing_exchange = (self.exchange_set.pop(),time(),self.exchange_number)
            self.send_message_without_data('exchange',self.ongoing_exchange[0],self.exchange_number)

        if self.ongoing_exchange is not None:
            if received:
                if typ == 'ack':
                    self.send_exchange_message(id,eid)
                    teamset = self.stabilize_teams(message)
                    if teamset is not None:
                        self.send_finish_exchange_message(self.ongoing_exchange[0],teamset,eid)
                        self.finish_exchange(teamset)
                        self.last_exchange[id] = time()
                        if id in self.exchange_set:
                            self.exchange_set.remove(id)
                        self.ongoing_exchange = None
                        return
                elif typ == 'wait':
                    self.ongoing_exchange = None
                    return
                elif typ == 'reject':
                    self.rejected_exchanges.add((id,time()))
                    self.ongoing_exchange = None
                    return
                elif typ == 'fin':
                    self.finish_exchange(((message.teams_tid,message.teams_id,message.teams_rol,message.teams_su),(message.suitability_robots,message.suitability_roles,message.suitability_values)))
                    self.last_exchange[id] = time()
                    if id in self.exchange_set:
                        self.exchange_set.remove(id)
                    self.ongoing_exchange = None
                    return
            if time()-self.ongoing_exchange[1] >= self.exchange_timeout:
                self.ongoing_exchange = None

    def process_team_change(self):
        if self.ongoing_exchange is not None or self.team_change_message is None: return
        msg = CommunicationMessage()
        id,rol,su = msg.teams_id,msg.teams_rol,msg.teams_su
        sid,srol,ssu = msg.suitability_robots,msg.suitability_roles,msg.suitability_values
        team = set()
        for i in range(len(id)):
            team.add((id[i],rol[i],su[i]))
        suitabilities = set()
        for i in range(len(sid)):
            suitabilities.add((sid[i],srol[i],ssu[i]))
        self.team = team
        self.suitabilities = suitabilities
        self.team_change_message = None

    def role_check(self):
        pass

    def callback(self):
        if (self._int is None or time()-self._int > 10):
            x  = {'leader':[],'mapper':[]}
            for id,rol,su in self.team:
                x[rol].append(id)
            print(self.id,':',x)
            self._int = time()
        
        self.send_message_without_data('discovery')
        self.process_inbox()
        self.trim_team()
        self.trim_last_contacted()
        self.trim_rejected_exchanges()
        self.exchange_protocol()
        self.process_team_change()