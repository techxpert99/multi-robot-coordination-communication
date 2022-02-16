from controller.auxiliary import wait
from controller.wrappers import ThreadWrapper
import numpy as np
import cv2

class Visualizer:

    def __init__(self,store):
        self._store = store
        store.set('visualizer','callbacks',[])
        self._wait = store.get('general','visualizer_node_callback_interval')
        self._tw = ThreadWrapper(self._callback,self._wait)
        self._tw.__thread__.setName(store.get('names','namespace')+'/visualizer')
        self.add_visualization_callback((store.get('visualizer','window_name'),self.critical_map_matrix_function))
        self._viz_data_dict = store.get('visualizer','viz_data_dict')
        self._tw.start()

    def _callback(self):
        self._store.lock('visualizer')
        callbacks = list(self._store.unsafe_get('visualizer','callbacks'))
        self._store.release('visualizer')
        for window_name,callback in callbacks:
            matrix = callback()
            if matrix is not None:
                self._viz_data_dict['lock'].acquire()
                self._viz_data_dict['window_name'] = window_name
                self._viz_data_dict['data'] = matrix.copy()
                self._viz_data_dict['lock'].release()
                wait(self._wait/len(callbacks))
    
    def destroy(self):
        self._tw.stop()

    def cv2_tf(self,point):
        return point[1],point[0]

    def rasterize_matrix(self,matrix,color_function):
        row,col = matrix.shape
        image = np.zeros((row,col,3),np.uint8)
        for i in range(row):
            for j in range(col):
                image[i,j] = color_function(matrix[i,j])
        return image
    
    def draw_path(self,image,path,color=(0,0,0),thickness=1):
        for k in range(1,len(path)):
            image = cv2.line(image,self.cv2_tf(path[k-1]),self.cv2_tf(path[k]),color,thickness)
        return image
    
    def fill_cells(self,image,cells,color=(0,0,0)):
        for cell in cells:
            image[cell] = color
        return image

    def draw_point(self,image,point,color=(0,0,0),radius=1):
        return cv2.circle(image,self.cv2_tf(point),radius,color,cv2.FILLED)
    
    def add_visualization_callback(self,matrix_function):
        self._store.lock('visualizer')
        self._store.unsafe_get('visualizer','callbacks').append(matrix_function)
        self._store.release('visualizer')

    def draw_global_path_locally(self,image,global_plan,region_id):
        start_cell = True
        goal_cell = None
        for id,cell in global_plan:
            if id == region_id:
                goal_cell = cell
                if start_cell:
                    image = cv2.circle(image,self.cv2_tf(cell),1,(0,255,255))
                    start_cell = False
                else:
                    image[cell] = (255,0,0)
        if goal_cell is not None:
            image = cv2.circle(image,self.cv2_tf(goal_cell),1,(0,255,0))
        return image

    def critical_map_matrix_function(self):

        if not self._store.has('estimator','position'):
            return None

        cx,cy,ct = self._store.get('estimator','position')

        self._store.lock('mapper')

        world = self._store.unsafe_get('mapper','world')
        region_id = world.region_id((cx,cy))
        region = world.get_region_by_id(region_id)
        matrix = region[1]

        def col_fn(x):
            if x == 0: return (0,0,0)
            elif x == 1: return (0,128,255)
            elif x == 2: return (255,255,0)
            elif x == 3: return (255,255,255)
            elif x == 5:
                return (200,100,150)
            return (128,128,128)

        current_cell = world.inverse_regional_transform(region[0],(cx,cy))
        image = self.rasterize_matrix(matrix,col_fn)

        self._store.release('mapper')

        image = self.draw_point(image,current_cell,(0,0,255))

        self._store.lock('planner')
        plan = self._store.unsafe_get('planner','plan')
        self._store.release('planner')

        if plan and len(plan) >= 2:
            image = self.draw_global_path_locally(image,plan,region_id)

        image = cv2.resize(image,self._store.get('general','visualized_map_size'))
        return image
    