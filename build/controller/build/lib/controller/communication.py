from robot_communication_interfaces.msg import CommunicationMessage
from controller.vector_clocks import SafeVectorClock
from controller.wrappers import NodeWrapper, ThreadWrapper
from time import time

class CommunicationNode:

    def __init__(self,store):
        self._nw = NodeWrapper(store.get('names','communication'),store)
        self._store = store
        self._clock = SafeVectorClock(store.get('names','namespace'))
        self._pub = self._nw.create_publisher('broadcasts')
        self._robot_id = store.get('names','namespace')
        self._exchange_interval = store.get('general','coordination_exchange_interval')
        self._nw.create_subscription('com.in',self._recv_cb)
        self._tw = ThreadWrapper(self._broadcast_cb,store.get('general','communication_broadcast_callback_interval'))
        self._tw.start()
        self._last_exchange = dict()

    def _broadcast_cb(self):
        if not self._store.has('estimator','position'): return
        position = self._store.get('estimator','position')
        self._clock.lock()
        self._clock.increment()
        clock = self._clock.serialize()
        self._clock.release()
        msg = CommunicationMessage()
        msg.robot_id = self._robot_id
        msg.message_type = 'DISCOVERY_BROADCAST'
        msg.clock_process_vector,msg.clock_time_vector = clock
        msg.position = position
        self._pub.publish(msg)
    
    def interrupt_coordinator(self,interrupt_type,sender_robot_id):
        self._store.lock('coordinator')
        interrupt_queue = self._store.unsafe_get('coordinator','interrupts')
        interrupt_queue.append({'type':interrupt_type,'robot_id':sender_robot_id})
        self._store.release('coordinator')
    
    def _recv_cb(self,msg:CommunicationMessage):
        self._clock.lock()
        self._clock.update((msg.clock_process_vector,msg.clock_time_vector))
        self._clock.increment()
        self._clock.release()
        local_clock_time = time()
        if msg.message_type == 'DISCOVERY_BROADCAST':
            if msg.robot_id not in self._last_exchange or local_clock_time-self._last_exchange[msg.robot_id] > self._exchange_interval:
                self._last_exchange[msg.robot_id] = local_clock_time
                self.interrupt_coordinator('encounter',msg.robot_id)
        elif msg.message_type == 'ENCOUNTER_EXCHANGE':
            self.interrupt_coordinator('exchange_requested',msg.robot_id)
        elif msg.message_type == 'REJECT_EXCHANGE':
            self.interrupt_coordinator('exchange_rejected',msg.robot_id)
        elif msg.message_type == 'WAIT_FOR_EXCHANGE':
            self.interrupt_coordinator('exchange_wait',msg.robot_id)
        elif msg._message_type == 'AGREED_FOR_EXCHANGE':
            self.interrupt_coordinator('exchange_accepted',msg.robot_id)
                    
    def send_message_no_data(self,receiver_robot_id,message_type):
        self._clock.lock()
        self._clock.increment()
        pv,tv = self._clock.serialize()
        self._clock.release()
        msg = CommunicationMessage()
        msg.robot_id = self._robot_id
        msg.receiver_robot_id = receiver_robot_id
        msg.message_type = message_type
        msg.clock_process_vector = pv
        msg.clock_time_vector = tv
        msg.position = self._store.get('estimator','position')
        self._pub.publish(msg)

    def request_exchange(self,receiver_robot_id):
        self.send_message_no_data(receiver_robot_id,'ENCOUNTER_EXCHANGE')
    
    def reject_exchange_request(self,receiver_robot_id):
        self.send_message_no_data(receiver_robot_id,'REJECT_EXCHANGE')        

    def wait_for_exchange(self,receiver_robot_id):
        self.send_message_no_data(receiver_robot_id,'WAIT_FOR_EXCHANGE')

    def allow_exchange(self,receiver_robot_id):
        self.send_message_no_data(receiver_robot_id,'AGREED_FOR_EXCHANGE')

    def destroy(self):
        self._tw.stop()
        self._nw.destroy()