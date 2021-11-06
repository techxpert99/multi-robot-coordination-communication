from time import time
from controller.auxiliary import wait
from controller.wrappers import ThreadWrapper


class CoordinationNode:

    def __init__(self,communicator,controller,store):
        self._store = store
        self._store.set('coordinator','interrupts',list())
        self._store.set('coordinator','encounter_set',set())
        self._store.set('coordinator','can_perform_encounter',True)
        self._store.set('coordinator','ongoing_encounter',None)
        self._store.set('coordinator','exchange_result',None)
        self._controller = controller
        self._communicator = communicator
        self._exchange_timeout = store.get('general','coordination_exchange_timeout')
        self._exchange_spin_interval = store.get('general','coordination_exchange_spin_interval')
        self._tw = ThreadWrapper(self._interrupt_cb,store.get('general','coordination_interrupt_handler_callback_interval'))
        self._etw = ThreadWrapper(self._encounter_cb,store.get('general','coordination_encounter_handler_callback_interval'))
        self._tw.start()
        self._etw.start()
    
    def _interrupt_cb(self):
        self._store.lock('coordinator')
        interrupt_list = self._store.unsafe_get('coordinator','interrupts')
        if len(interrupt_list) == 0:
            self._store.release('coordinator')
            return
        interruption = interrupt_list.pop(0)
        if interruption['type'] == 'encounter':
            encounter_set = self._store.unsafe_get('coordinator','encounter_set')
            if interruption['robot_id'] not in encounter_set:
                encounter_set.add(interruption['robot_id'])
        elif interruption['type'] == 'exchange_requested':
            ongoing = self._store.unsafe_get('coordinator','ongoing_encounter')
            if ongoing == interruption['robot_id']:
                self._store.release('coordinator')
                self._communicator.allow_exchange(interruption['robot_id'])
            else:
                if not self._store.unsafe_get('coordinator','can_perform_encounter'):
                    self._store.release('coordinator')
                    self._communicator.reject_exchange_request(interruption['robot_id'])
                else:
                    if ongoing is not None:
                        self._store.release('coordinator')
                        self._communicator.wait_for_exchange(interruption['robot_id'])
                    else:
                        self._store.unsafe_set('coordinator','ongoing_encounter',interruption['robot_id'])
                        encounter_set = self._store.unsafe_get('coordinator','encounter_set')
                        if interruption['robot_id'] in encounter_set:
                            encounter_set.remove(interruption['robot_id'])
                        self._store.release('coordinator')
            return
        elif interruption['type'] == 'exchange_rejected':
            self._store.unsafe_set('coordinator','exchange_result','reject')
        elif interruption['type'] == 'exchange_wait':
            self._store.unsafe_set('coordinator','exchange_result','wait')
        elif interruption['type'] == 'exchange_accepted':
            self._store.unsafe_set('coordinator','exchange_result','accept')
        self._store.release('coordinator')
    
    def _encounter_cb(self):
        if self._store.get('coordinator','ongoing_encounter') is None:
            self._store.lock('coordinator')
            encounter_set = self._store.unsafe_get('coordinator','encounter_set')
            doable = self._store.unsafe_get('coordinator','can_perform_encounter')
            if len(encounter_set) == 0 or not doable:
                self._store.release('coordinator')
                return
            encounter_robot_id = encounter_set.pop()
            self._store.unsafe_set('coordinator','ongoing_encounter',encounter_robot_id)
            self._store.release('coordinator')
        self.encounter()
    
    def encounter(self):
        self._store.set('critical','interrupt',True)
        self._controller.critical_stop()
        encounter_robot_id = self._store.get('coordinator','ongoing_encounter')
        self._communicator.request_exchange(encounter_robot_id)
        begin_time = time()
        accepted = False
        while time()-begin_time < self._exchange_timeout:
            result = self._store.get('coordinator','exchange_result')
            if result == 'reject':
                self._store.set('critical','interrupt',False)
                return
            elif result == 'wait':
                self._store.lock('coordinator')
                self._store.unsafe_get('coordinator','encounter_set').add(encounter_robot_id)
                self._store.release('coordinator')
                self._store.set('critical','interrupt',False)
                return
            elif result == 'accept':
                accepted = True
                break
            wait(self._exchange_spin_interval)
        self._store.set('coordinator','exchange_result',None)
        if not accepted:
            self._store.set('critical','interrupt',False)
            return
        print('encounter_initiated')
        wait(30)
        self._store.set('coordinator','ongoing_encounter',None)
        self._store.set('critical','interrupt',False)
        
    def destroy(self):
        self._tw.stop()
        self._etw.stop()
        pass